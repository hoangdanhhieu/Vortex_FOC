/**
 * @file foc_startup.c
 * @brief FOC Rotor Alignment, Open-Loop I-f Startup & Transition Blending Implementation
 */

#include "foc_startup.h"

#include <math.h>

#include "cordic_math.h"
#include "foc.h"
#include "foc_config.h"
#include "foc_state_machine.h"

/* Transition blending state (open-loop → closed-loop) */
static float s_blend_alpha = 0.0f; /* 0 = open-loop, 1 = closed-loop */
static uint32_t s_transition_counter = 0;
static uint32_t s_transition_samples = 0;
static uint8_t s_in_transition = 0;
static uint32_t s_handoff_converge_cnt = 0;
static uint32_t s_startup_stall_cnt = 0;
static uint32_t s_startup_timeout_cnt = 0;
static float s_delta_theta_flt = 0.0f;
static float s_delta_theta_variance = 1.0f;
static float s_handoff_id = 0.0f;

void FOC_Startup_Reset(void) {
    s_blend_alpha = 0.0f;
    s_transition_counter = 0;
    s_transition_samples = 0;
    s_in_transition = 0;
    s_handoff_converge_cnt = 0;
    s_startup_stall_cnt = 0;
    s_delta_theta_flt = 0.0f;
    s_delta_theta_variance = 1.0f;
    g_foc.data.e_real_flt = 0.0f;
    s_handoff_id = 0.0f;
    g_foc.status.in_transition = 0;
}

void FOC_Startup_ForceComplete(void) {
    s_blend_alpha = 1.0f;
    s_transition_counter = 0;
    s_transition_samples = 0;
    s_in_transition = 0;
    g_foc.status.in_transition = 0;
}

void FOC_Startup_StartTransition(float blend_duration_ms, float handoff_id) {
    s_handoff_id = handoff_id;
    s_blend_alpha = 0.0f;
    s_transition_counter = 0;
    s_transition_samples = (uint32_t)(blend_duration_ms * 0.001f * g_foc.cfg.pwm_frequency);
    if (s_transition_samples < 1) s_transition_samples = 1;
    s_in_transition = 1;
    g_foc.status.in_transition = 1;
}

void FOC_StateAlign(void) {
    if (g_foc.startup.counter == 0) {
        /* Ensure startup handoff speed respects physical observer minimum speed */
        if (g_foc.cfg.startup_handoff_speed < g_foc.cfg.motor_min_spd) {
            g_foc.cfg.startup_handoff_speed = g_foc.cfg.motor_min_spd;
        }
    }
    g_foc.startup.counter++;
    if (g_foc.startup.counter == 1) {
        PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
        PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);
    }

    float align_samples = (float)ALIGN_DURATION_MS * 0.001f * g_foc.cfg.pwm_frequency;
    float progress = (float)g_foc.startup.counter / align_samples;
    if (progress > 1.0f) progress = 1.0f;

    /* Ramp alignment current from 0 to align_current using Cosine S-Curve over first 70% duration.
     * Remaining 30% holds steady to settle any mechanical oscillations. */
    float ramp_duration = 0.7f;
    if (progress < ramp_duration) {
        float ramp_norm = progress / ramp_duration; /* 0.0f to 1.0f maps to 0 to pi */
        float cos_ramp, sin_ramp;
        cordic_sincos(ramp_norm, &cos_ramp, &sin_ramp);
        g_foc.cmd.Id_ref = 0.5f * (1.0f - cos_ramp) * g_foc.cfg.align_current;
    } else {
        g_foc.cmd.Id_ref = g_foc.cfg.align_current;
    }
    g_foc.cmd.Iq_ref = 0.0f;

    /* Fixed alignment angle at 0.0f (no artificial sweeping or snapping) */
    g_foc.data.theta_elec = 0.0f;
    g_foc.data.omega_elec = 0.0f;

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
        s_startup_timeout_cnt = 0;
        g_foc.startup.counter = 0;
        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        /* Do NOT reset PI controllers to preserve steady-state integrator voltage */
        g_foc.ctrl.smo.theta_est = 0.0f;
        g_foc.ctrl.smo.omega_est = 0.0f;
        g_foc.ctrl.smo.omega_out = 0.0f;
        g_foc.ctrl.smo.omega_stf = 0.0f;
        g_foc.ctrl.smo.pll_integral = 0.0f;
        g_foc.status.state = FOC_STATE_STARTUP;
    }
}

CCMRAM_FUNC void FOC_StateStartup(void) {
    g_foc.startup.counter++;

    float handoff_omega = g_foc.cfg.startup_handoff_speed;
    g_foc.startup.omega += g_foc.cfg.startup_accel * g_foc.dt;
    if (g_foc.startup.omega > handoff_omega) {
        g_foc.startup.omega = handoff_omega;
    }

    g_foc.data.omega_elec = g_foc.startup.omega;

    g_foc.startup.theta += (g_foc.startup.omega * g_foc.dt) / PI;
    g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);
    g_foc.data.theta_elec = g_foc.startup.theta;

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);
    park_transform(g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    /* Universal Vector Steering for Smooth Propeller Startup & Pole-Slip Immunity:
     * Steer current vector angle gamma_norm smoothly from 0 (all Id) to 1/3 (60 deg: 50% Id, 86.6%
     * Iq). Uses 0.35 * handoff_omega bounded to [30, 80] rad/s (5-13 Hz electrical) to
     * ensure breakaway completes locally without keeping excessive Id at high handoff speeds. */
    float steer_threshold = clampf(0.35f * handoff_omega, 30.0f, 80.0f);
    float steer_ratio = (steer_threshold > 1.0f) ? (g_foc.startup.omega / steer_threshold) : 1.0f;
    if (steer_ratio > 1.0f) steer_ratio = 1.0f;

    const float gamma_target_norm = 1.0f / 3.0f; /* 60 deg = (pi/3) / pi */
    float gamma_norm = steer_ratio * gamma_target_norm;

    float cos_gamma, sin_gamma;
    cordic_sincos(gamma_norm, &cos_gamma, &sin_gamma);

    float i_mag = g_foc.cfg.startup_current;
    g_foc.cmd.Id_ref = i_mag * cos_gamma;
    g_foc.cmd.Iq_ref = i_mag * sin_gamma;

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    /* Calculate voltage commands using closed-loop current PI controllers + Feedforward */
    float omega_e = g_foc.startup.omega;
    float omega_Ls = omega_e * g_foc.cfg.motor_ls;
    float E_bemf = omega_e * g_foc.cfg.motor_flux;

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
    float smo_theta = SMO_GetParkAngle(&g_foc.ctrl.smo);
    float delta_theta = normalize_angle_norm(smo_theta - g_foc.startup.theta);

    const float lpf_alpha = 50.0f * g_foc.dt;

    float diff_theta = normalize_angle_norm(delta_theta - s_delta_theta_flt);
    s_delta_theta_flt = normalize_angle_norm(s_delta_theta_flt + lpf_alpha * diff_theta);
    s_delta_theta_variance += lpf_alpha * (fabsf(diff_theta) - s_delta_theta_variance);
    if (g_foc.startup.omega >= handoff_omega * 0.8f) {
        if (s_delta_theta_variance < 0.05f) {
            s_handoff_converge_cnt++;
        } else {
            if (s_handoff_converge_cnt >= 2) {
                s_handoff_converge_cnt -= 2;
            }
        }
    }

    if (s_handoff_converge_cnt > HANDOFF_LOCK_SAMPLES) {
        SMO_ResetStates(&g_foc.ctrl.smo, g_foc.startup.omega);
        SMO_SlowTask(&g_foc.ctrl.smo);

        PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
        PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);

        /* Seamless torque continuity for all modes */
        float handoff_iq = g_foc.cmd.Iq_ref;
        g_foc.cmd.Iq_ref = handoff_iq;
        g_foc.data.Iq_ref_cmd = handoff_iq;

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = g_foc.startup.omega;
            if (fabsf(g_foc.cmd.speed_ref_target) < 1.0f ||
                g_foc.cmd.speed_ref_target < g_foc.startup.omega) {
                g_foc.cmd.speed_ref_target = g_foc.startup.omega;
            }
            LADRC_SeedState(&g_foc.ctrl.speed, g_foc.startup.omega, handoff_iq);
        }

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
        FOC_Startup_StartTransition(TRANSITION_BLEND_MS, g_foc.cmd.Id_ref);

        g_foc.status.state = FOC_STATE_RUN;
    }
#endif

#if STARTUP_TIMEOUT_MS > 0
    if (s_startup_timeout_cnt >
        (uint32_t)((STARTUP_TIMEOUT_MS * 0.001f * g_foc.cfg.pwm_frequency))) {
        g_foc.status.fault = FOC_FAULT_STARTUP_FAIL;
        g_foc.status.state = FOC_STATE_FAULT;
    } else {
        if (g_foc.startup.omega >= handoff_omega * 0.8f) {
            s_startup_timeout_cnt++;
        }
    }
#endif
}

CCMRAM_FUNC void FOC_Transition_Update(float smo_theta_park, float smo_theta_pwm, float smo_omega) {
    if (!s_in_transition) {
        return;
    }

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

    /* Smoothly blend Id_ref from handoff value down to 0 */
    g_foc.cmd.Id_ref = (1.0f - s_blend_alpha) * s_handoff_id;

    g_foc.startup.theta += (g_foc.startup.omega * g_foc.dt) / PI;
    g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);

    float delta_park = normalize_angle_norm(smo_theta_park - g_foc.startup.theta);
    g_foc.data.theta_park = normalize_angle_norm(g_foc.startup.theta + s_blend_alpha * delta_park);

    float delta_pwm = normalize_angle_norm(smo_theta_pwm - g_foc.startup.theta);
    g_foc.data.theta_elec = normalize_angle_norm(g_foc.startup.theta + s_blend_alpha * delta_pwm);

    g_foc.data.omega_elec =
        (1.0f - s_blend_alpha) * g_foc.startup.omega + s_blend_alpha * smo_omega;
}