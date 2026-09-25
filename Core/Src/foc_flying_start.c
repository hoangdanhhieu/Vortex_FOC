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

/* Active dynamic brake state (reverse spinning recovery) */
static uint32_t s_brake_counter = 0;
static uint32_t s_brake_min_samples = 0;
static uint32_t s_brake_max_samples = 0;
static uint32_t s_brake_debounce_target = 0;
static uint32_t s_brake_debounce_counter = 0;
static float s_brake_exit_curr_sq = 0.0f;
static float s_i_mag_sq_flt = 100.0f;
static float s_detect_peak_alpha;

/* Helper to extract pure BEMF fundamental by rejecting common-mode DC offset */
static inline void get_pure_bemf(float* alpha, float* beta) {
    float va = g_foc.data.Vphase_a;
    float vc = g_foc.data.Vphase_c;

    /* Direct 2-phase Clarke transform (since vb = -(va + vc)):
     * raw_alpha = (va - 0.5*vb - 0.5*vc) * (2/3) = va
     * raw_beta  = SQRT3_INV * (vb - vc) = -SQRT3_INV * (va + 2*vc) */
    float raw_alpha = va;
    float raw_beta = (g_foc.status.reverse > 0.0f) ? (-SQRT3_INV * (va + 2.0f * vc))
                                                   : (SQRT3_INV * (va + 2.0f * vc));

    /* Fast DC blocker (HPF) to remove offset caused by diode clamping and ADC clipping.
     * Cutoff ~15Hz*/
    if (s_detect_counter <= 1 && g_foc.status.state == FOC_STATE_DETECT) {
        s_dc_alpha = raw_alpha;
        s_dc_beta = raw_beta;
    } else {
        float alpha_dc = 15 * g_foc.dt * TWO_PI;
        s_dc_alpha += (raw_alpha - s_dc_alpha) * alpha_dc;
        s_dc_beta += (raw_beta - s_dc_beta) * alpha_dc;
    }

    *alpha = raw_alpha - s_dc_alpha;
    *beta = raw_beta - s_dc_beta;
}

void FOC_FlyingStart_Init(void) {
    /* 1. Physical minimum electrical speed determined by calibration [rad/s elec] */
    float omega_min_elec = g_foc.cfg.motor_min_spd;
    if (omega_min_elec < 30.0f) omega_min_elec = 30.0f;

    /* 2. Minimum electrical frequency for detection */
    float f_elec_min = omega_min_elec * (1.0f / TWO_PI);
    if (f_elec_min < 10.0f) f_elec_min = 10.0f; /* Safety baseline: >= 10 Hz */

    /* 3. Detect duration: Observe at least 2 electrical cycles, clamped between [50ms, 150ms] */
    float t_detect = 2.0f / f_elec_min;
    if (t_detect < 0.05f) t_detect = 0.05f;
    if (t_detect > 0.15f) t_detect = 0.15f;
    s_detect_samples = (uint32_t)(t_detect * g_foc.cfg.pwm_frequency);

    /* 4. Lock duration: 40 ms is optimal for PLL angle lock and noise filtering */
    s_lock_samples = (uint32_t)(0.040f * g_foc.cfg.pwm_frequency);

    /* 5. Peak phase BEMF threshold:
     * Derived directly from omega_min_elec and motor flux (E_phase = omega_elec * flux).
     * Set threshold to 60% of min BEMF for responsive detection */
    s_bemf_threshold = (omega_min_elec * g_foc.cfg.motor_flux) * 0.6f;

    /* Clamp floor to 50 mV (well above 15-20mV ADC noise floor, but sensitive to hand spinning) */
    if (s_bemf_threshold < 0.05f) s_bemf_threshold = 0.05f;

    s_detect_counter = 0;
    s_detect_peak = 0.0f;
    s_flying_start_counter = 0;

    s_E_alpha_prev = 0.0f;
    s_E_beta_prev = 0.0f;
    s_cross_prod_sum = 0.0f;

    s_dc_alpha = 0.0f;
    s_dc_beta = 0.0f;

    s_detect_peak_alpha = 400.0f * TWO_PI * FOC_GetDt();

    /* Braking timing initialization */
    s_brake_min_samples =
        (uint32_t)((float)BRAKE_MIN_DURATION_MS * 0.001f * g_foc.cfg.pwm_frequency);
    s_brake_max_samples =
        (uint32_t)((float)BRAKE_MAX_DURATION_MS * 0.001f * g_foc.cfg.pwm_frequency);
    s_brake_debounce_target =
        (uint32_t)((float)BRAKE_DEBOUNCE_MS * 0.001f * g_foc.cfg.pwm_frequency);
    s_brake_counter = 0;
    s_brake_debounce_counter = 0;
    float exit_curr = BRAKE_EXIT_CURR_MIN;
    s_brake_exit_curr_sq = exit_curr * exit_curr;
    s_i_mag_sq_flt = 100.0f;
}

void FOC_StateDetect(void) {
    s_detect_counter++;
    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    float E_alpha, E_beta;
    get_pure_bemf(&E_alpha, &E_beta);

    float amp = E_alpha * E_alpha + E_beta * E_beta;

    /* Low-Pass Filter on amplitude squared (DC value) to reject noise spikes */
    s_detect_peak += s_detect_peak_alpha * (amp - s_detect_peak);

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

            s_flying_start_counter = 0;
            g_foc.status.state = FOC_STATE_FLYING_START;
        } else {
            FOC_EnableDrivers(1);
            g_foc.status.state = FOC_STATE_ALIGN;
        }
    }
}

RAM_FUNC void FOC_StateFlyingStart(void) {
    s_flying_start_counter++;

    float E_alpha, E_beta;
    get_pure_bemf(&E_alpha, &E_beta);

    SMO_FeedBEMF(&g_foc.ctrl.smo, E_alpha, E_beta);

    float omega_now = g_foc.ctrl.smo.omega_est;
    g_foc.data.omega_elec = omega_now;

    if (s_flying_start_counter < s_lock_samples) {
        g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;
        return;
    }

    /* Check exit criteria before precharge / enable */
    if (omega_now <= 0.0f) {
        FOC_EnableDrivers(1);
        s_brake_counter = 0;
        s_brake_debounce_counter = 0;
        s_i_mag_sq_flt = 100.0f;
        g_foc.status.state = FOC_STATE_BRAKE;
        return;
    }

    if (omega_now < g_foc.cfg.motor_min_spd * 0.6f) {
        FOC_EnableDrivers(1);
        g_foc.startup.counter = 0;
        g_foc.status.state = FOC_STATE_ALIGN;
        return;
    }

    /* Common voltage and PWM duty calculation for both Precharge and Enable cycles:
     * - Cycle s_lock_samples (Precharge): Drivers remain disabled. CCR preload registers are
     * written. Timer update event latches duty cycles into shadow registers at the next valley.
     * - Cycle s_lock_samples + 1 (Enable): Drivers enabled with matching PWM voltage on the motor.
     */
    float theta_park = SMO_GetParkAngle(&g_foc.ctrl.smo);

    float sin_th, cos_th;
    cordic_sincos(theta_park, &cos_th, &sin_th);
    float Ed, Eq;
    park_transform(E_alpha, E_beta, cos_th, sin_th, &Ed, &Eq);

    /* Use actual measured BEMF (Eq) instead of theoretical (omega * flux) */
    float E_bemf = Eq;

    g_foc.data.Vd = 0.0f;
    g_foc.data.Vq = E_bemf;

    float theta_pwm = SMO_GetPWMAngle(&g_foc.ctrl.smo);
    if (g_foc.cfg.comp_delay_samples > 0.001f) {
        theta_pwm += omega_now * (g_foc.cfg.comp_delay_samples * CONTROL_PERIOD) / PI;
        theta_pwm = normalize_angle_norm(theta_pwm);
    }

    svpwm_calculate(theta_pwm);

    if (s_flying_start_counter > s_lock_samples) {
        /* Pre-load current PI integrals for bumpless transfer (FOC_StateRun FF provides 0.9 *
         * E_bemf) */
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        LADRC_Reset(&g_foc.ctrl.speed);

        const float ff_gain = 0.9f;
        g_foc.ctrl.id.integral = 0.0f;
        g_foc.ctrl.iq.integral = E_bemf * (1.0f - ff_gain);

        /* Seed SMO current observer using inverse sigmoid so the observer starts ALREADY
         * on the sliding surface. This prevents BEMF from collapsing to 0.0V (zero-sigmoid shock)
         * on cycle 1 of RUN. */
        float k_slide = g_foc.ctrl.smo.k_slide;
        float k_sigmoid = g_foc.ctrl.smo.k_sigmoid;
        if (k_slide > 0.1f) {
            float inv_ks = 1.0f / k_slide;
            float ya = clampf(g_foc.ctrl.smo.Ealpha_flt * inv_ks, -0.95f, 0.95f);
            float yb = clampf(g_foc.ctrl.smo.Ebeta_flt * inv_ks, -0.95f, 0.95f);
            float err_a = (k_sigmoid * ya) / (1.0f - fabsf(ya));
            float err_b = (k_sigmoid * yb) / (1.0f - fabsf(yb));
            g_foc.ctrl.smo.Ialpha_est = g_foc.data.Ialpha + err_a;
            g_foc.ctrl.smo.Ibeta_est = g_foc.data.Ibeta + err_b;
        } else {
            g_foc.ctrl.smo.Ialpha_est = g_foc.data.Ialpha;
            g_foc.ctrl.smo.Ibeta_est = g_foc.data.Ibeta;
        }

        /* Seed SMO raw BEMF states using clean STF-filtered BEMF vector */
        g_foc.ctrl.smo.Ealpha = g_foc.ctrl.smo.Ealpha_flt;
        g_foc.ctrl.smo.Ebeta = g_foc.ctrl.smo.Ebeta_flt;

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = omega_now;
            if (fabsf(g_foc.cmd.speed_ref_target) < 1.0f ||
                g_foc.cmd.speed_ref_target < omega_now) {
                g_foc.cmd.speed_ref_target = omega_now;
            }
            /* Bumpless zero-torque handoff: motor is freewheeling, so initial torque command is 0.
             * LADRC seamlessly ramps torque according to the user throttle target without an
             * initial current kick. */
            g_foc.cmd.Iq_ref = 0.0f;
            g_foc.data.Iq_ref_cmd = 0.0f;
            LADRC_SeedState(&g_foc.ctrl.speed, omega_now, 0.0f);
        } else if (g_foc.status.control_mode == FOC_MODE_TORQUE) {
            g_foc.data.Iq_ref_cmd = g_foc.cmd.Iq_ref;
        } else {
            g_foc.cmd.Iq_ref = 0.0f;
            g_foc.data.Iq_ref_cmd = 0.0f;
        }
        g_foc.cmd.Id_ref = 0.0f;
        if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            float max_v = SQRT3_INV * 2.0f * (g_foc.max_duty - 0.5f) * g_foc.data.Vbus;
            float E_bemf_norm = (max_v > 1.0f) ? (E_bemf / max_v) : 0.0f;
            E_bemf_norm = clampf(E_bemf_norm, -1.0f, 1.0f);
            g_foc.cmd.Vq_ref = E_bemf_norm;
        }

        /* Direct handoff: bypass startup angle blending */
        g_foc.startup.theta = theta_park;
        g_foc.startup.omega = omega_now;
        FOC_Startup_ForceComplete();
        FOC_EnableDrivers(1);
        g_foc.status.state = FOC_STATE_RUN;
    }
}

void FOC_StateBrake(void) {
    s_brake_counter++;
    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.0f;

    /* Current magnitude squared in stationary alpha-beta frame */
    float i_mag_sq =
        g_foc.data.Ialpha_flt * g_foc.data.Ialpha_flt + g_foc.data.Ibeta_flt * g_foc.data.Ibeta_flt;

    s_i_mag_sq_flt += 0.01f * (i_mag_sq - s_i_mag_sq_flt);

    /* Check exit criteria */
    if (s_brake_counter >= s_brake_max_samples) {
        /* Maximum timeout expired: fail-safe exit to ALIGN */
        g_foc.startup.counter = 0;
        g_foc.status.state = FOC_STATE_ALIGN;
    } else if (s_brake_counter >= s_brake_min_samples && s_i_mag_sq_flt < s_brake_exit_curr_sq) {
        /* Below noise threshold: count consecutive debounce samples */
        s_brake_debounce_counter++;
        if (s_brake_debounce_counter >= s_brake_debounce_target) {
            g_foc.startup.counter = 0;
            g_foc.status.state = FOC_STATE_ALIGN;
        }
    } else {
        s_brake_debounce_counter = 0;
    }
}
