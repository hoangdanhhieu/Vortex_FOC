/**
 * @file foc_startup.c
 * @brief FOC Rotor Alignment, Open-Loop I-f Startup & Transition Blending Implementation
 */

#include "foc_startup.h"

#include "cordic_math.h"
#include "foc.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "math.h"
#include "motor_id.h"
#include "peripheral_init.h"

/* Transition blending state (open-loop → closed-loop) */
static float s_blend_alpha = 0.0f; /* 0 = open-loop, 1 = closed-loop */
static uint32_t s_transition_counter = 0;
static uint32_t s_transition_samples = 0;
static uint8_t s_in_transition = 0;
static uint32_t s_closed_loop_counter = 0;

void FOC_Startup_Reset(void) {
    s_blend_alpha = 0.0f;
    s_transition_counter = 0;
    s_transition_samples = 0;
    s_in_transition = 0;
    s_closed_loop_counter = 0;
    g_foc.status.in_transition = 0;
}

void FOC_Startup_ForceComplete(void) {
    s_blend_alpha = 1.0f;
    s_transition_counter = 0;
    s_transition_samples = 0;
    s_in_transition = 0;
    g_foc.status.in_transition = 0;
}

void FOC_ResetClosedLoopCounter(void) {
    s_closed_loop_counter = 0;
}

uint8_t FOC_IsInTransition(void) {
    return s_in_transition;
}

void FOC_StateAlign(void) {
    g_foc.startup.counter++;
    if (g_foc.startup.counter == 1) {
        /* Calculate dynamic startup PI gains based on 4x Handoff Speed and apply them once at start
         * of Align */
        float handoff_omega = g_foc.cfg.startup_handoff_speed * RPM_TO_RAD * g_foc.cfg.motor_poles;
        float bw_startup = 4.0f * handoff_omega;
        float kp_startup = g_foc.cfg.motor_ls * bw_startup;
        float ki_startup = g_foc.cfg.motor_rs * bw_startup;
        PI_SetGains(&g_foc.ctrl.id, kp_startup, ki_startup);
        PI_SetGains(&g_foc.ctrl.iq, kp_startup, ki_startup);
    }

    float align_samples = (float)ALIGN_DURATION_MS * 0.001f * (float)CONTROL_FREQUENCY;
    float progress = (float)g_foc.startup.counter / align_samples;
    if (progress > 1.0f) progress = 1.0f;

    /* 1. Ramp alignment current from 0 to align_current over the first 30% of alignment duration */
    float ramp_duration = 0.3f;
    if (progress < ramp_duration) {
        g_foc.cmd.Id_ref = (progress / ramp_duration) * g_foc.cfg.align_current;
    } else {
        g_foc.cmd.Id_ref = g_foc.cfg.align_current;
    }
    g_foc.cmd.Iq_ref = 0.0f;

    /* 2. Smooth angle sweep from -45 degrees (-0.25 normalized) to 0 degrees (0.0 normalized)
     * to break any 180-degree electrical angle dead spots without mechanical impact. */
    float start_angle = -0.25f;  // -45 degrees electrical
    g_foc.data.theta_elec = start_angle * (1.0f - progress);

    g_foc.data.omega_elec = 0.0f;
    g_foc.data.speed_rpm = 0.0f;

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);
    park_transform(g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error);
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error);

    svpwm_calculate(g_foc.data.theta_elec);

    if (g_foc.startup.counter > (uint32_t)align_samples) {
        g_foc.startup.counter = 0;
        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        g_foc.ctrl.smo.theta_est = 0.0f;
        g_foc.ctrl.smo.omega_est = 0.0f;
        g_foc.ctrl.smo.omega_out = 0.0f;
        g_foc.ctrl.smo.omega_stf = 0.0f;
        g_foc.ctrl.smo.pll_integral = 0.0f;
        g_foc.status.state = FOC_STATE_STARTUP;
    }
}

void FOC_StateStartup(void) {
    g_foc.startup.counter++;

    float accel_rad = g_foc.cfg.startup_accel * RPM_TO_RAD * g_foc.cfg.motor_poles;
    g_foc.startup.omega += accel_rad * CONTROL_PERIOD;

    float handoff_omega = g_foc.cfg.startup_handoff_speed * RPM_TO_RAD * g_foc.cfg.motor_poles;
    if (g_foc.startup.omega > handoff_omega) {
        g_foc.startup.omega = handoff_omega;
    }

    g_foc.data.omega_elec = g_foc.startup.omega;
    g_foc.data.speed_rpm = (g_foc.startup.omega / g_foc.cfg.motor_poles) * (60.0f / TWO_PI);

    g_foc.startup.theta += (g_foc.startup.omega * CONTROL_PERIOD) / PI;
    g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);
    g_foc.data.theta_elec = g_foc.startup.theta;

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);
    park_transform(g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    g_foc.cmd.Id_ref = 0.0f;

    float startup_ramp_samples = 0.3f * (float)CONTROL_FREQUENCY;
    if ((float)g_foc.startup.counter < startup_ramp_samples) {
        float ratio = (float)g_foc.startup.counter / startup_ramp_samples;
        g_foc.cmd.Iq_ref = ratio * g_foc.cfg.startup_current;
    } else {
        g_foc.cmd.Iq_ref = g_foc.cfg.startup_current;
    }

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    /* Calculate voltage commands using closed-loop current PI controllers + Feedforward */
    float omega_e = g_foc.startup.omega;
    float omega_Ls = omega_e * g_foc.cfg.motor_ls;
    float E_bemf = omega_e * g_foc.cfg.motor_flux;
    
    if (MotorID_IsFluxMeasuring()) {
        omega_Ls = 0.0f;
        E_bemf = 0.0f;
    }

    g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq;
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + omega_Ls * g_foc.data.Id + E_bemf;

    /* Clamp (Vd, Vq) vector magnitude to 0.57735 * Vbus (SVPWM linear limit) */
    float max_v = SQRT3_INV * g_foc.data.Vbus;
    float v_sq = g_foc.data.Vd * g_foc.data.Vd + g_foc.data.Vq * g_foc.data.Vq;
    if (v_sq > max_v * max_v && v_sq > 1e-6f) {
        float inv_v = max_v / sqrtf(v_sq);
        g_foc.data.Vd *= inv_v;
        g_foc.data.Vq *= inv_v;
    }

    /* Inverse Park */
    svpwm_calculate(g_foc.data.theta_elec);

    SMO_Update(&g_foc.ctrl.smo, g_foc.data.Valpha, g_foc.data.Vbeta, g_foc.data.Ialpha,
               g_foc.data.Ibeta);

#if ENABLE_CLOSED_LOOP_HANDOFF
    if (g_foc.startup.omega >= handoff_omega * 0.9f) {
        if (MotorID_IsFluxMeasuring()) {
            g_foc.status.state = FOC_STATE_COAST_FLUX_ID;
            return;
        }

        SMO_ResetStates(&g_foc.ctrl.smo, g_foc.startup.omega);
        SMO_SlowTask(&g_foc.ctrl.smo);

        /* Restore original default PI gains from config */
        PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
        PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = g_foc.startup.omega;
            LADRC_SeedState(&g_foc.ctrl.speed, g_foc.startup.omega, 0.0f);
            g_foc.cmd.Iq_ref = 0.0f;
        } else {
            g_foc.cmd.Iq_ref = g_foc.data.Iq;
        }
        g_foc.data.Iq_ref_cmd = g_foc.cmd.Iq_ref;

        if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            float max_v = SQRT3_INV * g_foc.data.Vbus;
            float Vq_norm = (max_v > 1.0f) ? (g_foc.data.Vq / max_v) : 0.0f;
            Vq_norm = clampf(Vq_norm, 0.0f, 1.0f);

            float vq_min = (Vq_norm < 0.05f) ? 0.05f : Vq_norm;
            extern void FOC_Input_SetMinVq(float vq_min);
            FOC_Input_SetMinVq(vq_min);

            g_foc.cmd.Vq_ref = Vq_norm;
            g_foc.cmd.Vq_ref_target = Vq_norm;
        }
        s_blend_alpha = 0.0f;
        s_transition_counter = 0;
        s_transition_samples = (uint32_t)(TRANSITION_BLEND_MS * 0.001f * (float)CONTROL_FREQUENCY);
        if (s_transition_samples < 1) s_transition_samples = 1;
        s_in_transition = 1;
        g_foc.status.in_transition = 1;

        g_foc.cmd.Id_ref = 0.0f;
        g_foc.status.state = FOC_STATE_RUN;
    }
#endif

    /* Timeout check (only if enabled) */
#if STARTUP_TIMEOUT_MS > 0
    if (g_foc.startup.counter > (uint32_t)((STARTUP_TIMEOUT_MS * CONTROL_FREQUENCY) / 1000)) {
        g_foc.status.fault = FOC_FAULT_STARTUP_FAIL;
        g_foc.status.state = FOC_STATE_FAULT;
    }
#endif
}

CCMRAM_FUNC void FOC_Transition_Update(float smo_theta_park, float smo_theta_pwm, float smo_omega, float smo_speed_rpm) {
    if (s_in_transition) {
        s_transition_counter++;
        s_blend_alpha = (float)s_transition_counter / (float)s_transition_samples;

        if (s_blend_alpha >= 1.0f) {
            s_blend_alpha = 1.0f;
            s_in_transition = 0;
            g_foc.status.in_transition = 0;

            /* Restore original default PI gains from config */
            PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
            PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);
        }

        g_foc.startup.theta += (g_foc.startup.omega * CONTROL_PERIOD) / PI;
        g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);
        
        float delta_park = normalize_angle_norm(smo_theta_park - g_foc.startup.theta);
        g_foc.data.theta_park = normalize_angle_norm(g_foc.startup.theta + s_blend_alpha * delta_park);
        
        float delta_pwm = normalize_angle_norm(smo_theta_pwm - g_foc.startup.theta);
        g_foc.data.theta_elec = normalize_angle_norm(g_foc.startup.theta + s_blend_alpha * delta_pwm);
        
        g_foc.data.omega_elec =
            (1.0f - s_blend_alpha) * g_foc.startup.omega + s_blend_alpha * smo_omega;
        g_foc.data.speed_rpm = (1.0f - s_blend_alpha) * (g_foc.startup.omega /
                                                         g_foc.cfg.motor_poles * (60.0f / TWO_PI)) +
                               s_blend_alpha * smo_speed_rpm;
    } else {
        g_foc.data.theta_park = smo_theta_park;
        g_foc.data.theta_elec = smo_theta_pwm;
        g_foc.data.omega_elec = smo_omega;
        g_foc.data.speed_rpm = smo_speed_rpm;
    }
}
