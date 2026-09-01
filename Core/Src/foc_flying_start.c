/**
 * @file foc_flying_start.c
 * @brief FOC Flying Start, BEMF Tracking & Spinning PLL Lock Implementation
 */

#include "foc_flying_start.h"

#include "cordic_math.h"
#include "foc.h"
#include "foc_config.h"
#include "foc_startup.h"
#include "foc_state_machine.h"
#include "math.h"
#include "peripheral_init.h"

/* Flying start state */
static uint32_t s_detect_counter = 0;
static uint32_t s_detect_samples = 0;
static float s_detect_peak = 0.0f;
static float s_bemf_threshold = 0.0f;
static uint32_t s_flying_start_counter = 0;
static uint32_t s_lock_samples = 0;

static float s_E_alpha_prev = 0.0f;
static float s_E_beta_prev = 0.0f;
static float s_cross_prod_sum = 0.0f;
static float s_dc_alpha = 0.0f;
static float s_dc_beta = 0.0f;

/* Helper to extract pure BEMF fundamental by rejecting common-mode DC offset */
static inline void get_pure_bemf(float* alpha, float* beta) {
    float Ea = g_foc.data.Vphase_a;
    float Eb = g_foc.data.Vphase_b;
    float Ec = g_foc.data.Vphase_c;

    float raw_alpha = (Ea - 0.5f * Eb - 0.5f * Ec) * TWO_THIRDS;
    float raw_beta = SQRT3_INV * (Eb - Ec);

    /* Fast DC blocker (HPF) to remove the massive offset caused by diode clamping and ADC clipping.
     * Cutoff ~15Hz at 48kHz (alpha = 2 * pi * 15 * 20.8us = 0.002) */
    if (s_detect_counter <= 1 && g_foc.status.state == FOC_STATE_DETECT) {
        s_dc_alpha = raw_alpha;
        s_dc_beta = raw_beta;
    } else {
        float alpha_dc = 0.002f;
        s_dc_alpha += (raw_alpha - s_dc_alpha) * alpha_dc;
        s_dc_beta += (raw_beta - s_dc_beta) * alpha_dc;
    }

    *alpha = raw_alpha - s_dc_alpha;
    *beta = raw_beta - s_dc_beta;
}

void FOC_FlyingStart_Init(void) {
    float f_elec_min = g_foc.cfg.motor_min_spd * g_foc.cfg.motor_poles / 60.0f;
    if (f_elec_min < 1.0f) f_elec_min = 1.0f;
    s_detect_samples = (uint32_t)(2.0f / f_elec_min * (float)CONTROL_FREQUENCY);
    s_lock_samples = (uint32_t)(5.0f / (TWO_PI * SMO_PPL_CUTOFF) * (float)CONTROL_FREQUENCY);
    uint32_t min_lock_samples = (uint32_t)(0.05f * (float)CONTROL_FREQUENCY);  // 50 ms
    if (s_lock_samples < min_lock_samples) s_lock_samples = min_lock_samples;
    s_bemf_threshold = g_foc.cfg.motor_min_spd / g_foc.cfg.motor_kv * 0.8f;
    if (s_bemf_threshold < 0.05f) s_bemf_threshold = 0.05f;

    s_detect_counter = 0;
    s_detect_peak = 0.0f;
    s_flying_start_counter = 0;

    s_E_alpha_prev = 0.0f;
    s_E_beta_prev = 0.0f;
    s_cross_prod_sum = 0.0f;

    s_dc_alpha = 0.0f;
    s_dc_beta = 0.0f;
}

void FOC_StateDetect(void) {
    s_detect_counter++;
    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    float E_alpha, E_beta;
    get_pure_bemf(&E_alpha, &E_beta);

    float amp = E_alpha * E_alpha + E_beta * E_beta;

    /* Low-Pass Filter on amplitude squared (DC value) to reject noise spikes */
    s_detect_peak += 0.05f * (amp - s_detect_peak);

    /* Cross-product for direction detection */
    float cross = s_E_alpha_prev * E_beta - s_E_beta_prev * E_alpha;
    s_cross_prod_sum += cross;
    s_E_alpha_prev = E_alpha;
    s_E_beta_prev = E_beta;

    if (s_detect_counter >= s_detect_samples) {
        float thr_sq = s_bemf_threshold * s_bemf_threshold;
        if (s_detect_peak > thr_sq) {
            SMO_Reset(&g_foc.ctrl.smo);

            float E_amp_real = sqrtf(s_detect_peak);
            float est_omega = E_amp_real / g_foc.cfg.motor_flux;

            /* Apply detected direction */
            float sign_w = (s_cross_prod_sum >= 0.0f) ? 1.0f : -1.0f;
            est_omega *= sign_w;

            if (fabsf(est_omega) > g_foc.ctrl.smo.pll_int_max) {
                est_omega =
                    (est_omega > 0.0f) ? g_foc.ctrl.smo.pll_int_max : -g_foc.ctrl.smo.pll_int_max;
            }

            float theta_init = cordic_atan2(-E_alpha * sign_w, E_beta * sign_w);
            g_foc.ctrl.smo.theta_est = theta_init;

            g_foc.ctrl.smo.omega_est = est_omega;
            g_foc.ctrl.smo.omega_out = est_omega;
            g_foc.ctrl.smo.pll_integral = est_omega;
            g_foc.data.omega_elec = est_omega;
            g_foc.data.speed_rpm = (est_omega / g_foc.cfg.motor_poles) * (60.0f / TWO_PI);

            s_flying_start_counter = 0;
            g_foc.status.state = FOC_STATE_FLYING_START;
        } else {
            FOC_SetPhaseVoltageDMA(0);
            FOC_EnableDrivers(1);
            g_foc.status.state = FOC_STATE_ALIGN;
        }
    }
}

void FOC_StateFlyingStart(void) {
    s_flying_start_counter++;

    float E_alpha, E_beta;
    get_pure_bemf(&E_alpha, &E_beta);

    SMO_FeedBEMF(&g_foc.ctrl.smo, E_alpha, E_beta);

    float omega_now = g_foc.ctrl.smo.omega_est;
    g_foc.data.omega_elec = omega_now;
    g_foc.data.speed_rpm = (omega_now / g_foc.cfg.motor_poles) * (60.0f / TWO_PI);

    if (s_flying_start_counter < s_lock_samples) {
        g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    } else if (s_flying_start_counter == s_lock_samples) {
        float theta_park = SMO_GetParkAngle(&g_foc.ctrl.smo);

        float sin_th, cos_th;
        cordic_sincos(theta_park, &cos_th, &sin_th);
        float Ed, Eq;
        park_transform(E_alpha, E_beta, cos_th, sin_th, &Ed, &Eq);

        /* Use actual measured BEMF (Ed, Eq) instead of theoretical (omega * flux) */
        float E_bemf = Eq;

        g_foc.data.Vd = Ed;
        g_foc.data.Vq = E_bemf;

        /* Calculate exact same PWM angle used in FOC_StateRun to prevent angle jump */
        float theta_pwm = SMO_GetPWMAngle(&g_foc.ctrl.smo);
        if (g_foc.cfg.comp_delay_samples > 0.001f) {
            theta_pwm += omega_now * (g_foc.cfg.comp_delay_samples * CONTROL_PERIOD) / PI;
            theta_pwm = normalize_angle_norm(theta_pwm);
        }

        svpwm_calculate(theta_pwm);

    } else {
        /* -----------------------------------------------------------
         * ENABLE CYCLE: The timer update event between the previous
         * ISR and this one has loaded the correct duty from the
         * precharge cycle into the CCR shadow register. The PWM output
         * NOW matches back-EMF. Safe to enable gate drivers — the
         * very first PWM pulse the motor sees is correct.
         * ----------------------------------------------------------- */
        float theta_park = SMO_GetParkAngle(&g_foc.ctrl.smo);

        float sin_th, cos_th;
        cordic_sincos(theta_park, &cos_th, &sin_th);
        float Ed, Eq;
        park_transform(E_alpha, E_beta, cos_th, sin_th, &Ed, &Eq);

        /* Use actual measured BEMF (Eq) instead of theoretical (omega * flux) */
        float E_bemf = Eq;

        /* Recompute SVPWM for the updated angle (one cycle later) */
        g_foc.data.Vd = 0.0f;
        g_foc.data.Vq = E_bemf;

        float theta_pwm = SMO_GetPWMAngle(&g_foc.ctrl.smo);
        if (g_foc.cfg.comp_delay_samples > 0.001f) {
            theta_pwm += omega_now * (g_foc.cfg.comp_delay_samples * CONTROL_PERIOD) / PI;
            theta_pwm = normalize_angle_norm(theta_pwm);
        }

        svpwm_calculate(theta_pwm);

        /* Pre-load current PI integrals for bumpless transfer (compensate for FOC_StateRun FF) */
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        LADRC_Reset(&g_foc.ctrl.speed);

        g_foc.ctrl.iq.integral = E_bemf * 0.5;

        /* Seed SMO current observer with actual measured currents */
        g_foc.ctrl.smo.Ialpha_est = g_foc.data.Ialpha;
        g_foc.ctrl.smo.Ibeta_est = g_foc.data.Ibeta;

        /* Seed SMO raw BEMF states using the clean STF-filtered BEMF vector.
         * Do not use theta_park here, as it contains a PLL phase advance compensation
         * which would inject a phase mismatch into the observer states! */
        g_foc.ctrl.smo.Ealpha = g_foc.ctrl.smo.Ealpha_flt;
        g_foc.ctrl.smo.Ebeta = g_foc.ctrl.smo.Ebeta_flt;

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = omega_now;
            if (fabsf(g_foc.cmd.speed_ref_target) < 1.0f ||
                g_foc.cmd.speed_ref_target < omega_now) {
                g_foc.cmd.speed_ref_target = omega_now;
            }
            float iq_hand_off = g_foc.cfg.startup_current;
            float iq_min = 0.8f;
            float iq_max = g_foc.cfg.motor_max_curr * 0.15f;
            if (iq_max < iq_min) iq_max = iq_min;
            iq_hand_off = clampf(iq_hand_off, iq_min, iq_max);
            g_foc.cmd.Iq_ref = iq_hand_off;
            g_foc.data.Iq_ref_cmd = iq_hand_off;
            LADRC_SeedState(&g_foc.ctrl.speed, omega_now, iq_hand_off);
        } else if (g_foc.status.control_mode == FOC_MODE_TORQUE) {
            g_foc.data.Iq_ref_cmd = g_foc.cmd.Iq_ref;
        } else {
            g_foc.cmd.Iq_ref = 0.0f;
            g_foc.data.Iq_ref_cmd = 0.0f;
        }
        g_foc.cmd.Id_ref = 0.0f;
        if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            float max_v = SQRT3_INV * 2.0f * (MAX_DUTY_HIGH - 0.5f) * g_foc.data.Vbus;
            float E_bemf_norm = (max_v > 1.0f) ? (E_bemf / max_v) : 0.0f;
            E_bemf_norm = clampf(E_bemf_norm, -1.0f, 1.0f);
            g_foc.cmd.Vq_ref = E_bemf_norm;
        }

        /* Transition blending: bypass for flying start since the observer is already
         * locked and aligned. Angle blending can cause phase drift and current spikes. */
        g_foc.startup.theta = theta_park;
        g_foc.startup.omega = omega_now;
        FOC_Startup_ForceComplete();

        FOC_SetPhaseVoltageDMA(0);
        FOC_EnableDrivers(1);
        g_foc.status.state = FOC_STATE_RUN;
    }
}
