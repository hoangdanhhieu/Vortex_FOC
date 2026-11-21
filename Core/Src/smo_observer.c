/**
 * @file smo_observer.c
 * @brief Sliding Mode Observer implementation
 *
 * Uses a vector-norm continuous sigmoid function and exact discrete ZOH pole
 * placement so phase-lag compensation remains consistent across different motors.
 */

#include "smo_observer.h"

#include <math.h>

#include "cordic_math.h"
#include "foc.h"
#include "foc_config.h"

static void SMO_ComputeAndCommitGains(SMO_Observer_t* smo, float spd_ref, float vbus,
                                      uint8_t use_lpf) {
    float spd_abs = fabsf(spd_ref);
    float new_omega_filt;
    if (use_lpf) {
        if (smo->omega_est_filt == 0.0f && spd_abs > 1.0f) {
            new_omega_filt = spd_abs;
        } else {
            new_omega_filt = smo->omega_est_filt + 0.2f * (spd_abs - smo->omega_est_filt);
        }
    } else {
        new_omega_filt = spd_abs;
    }

    float pll_cutoff =
        clampf(smo->pll_alpha * new_omega_filt, smo->pll_cutoff_min, smo->pll_cutoff_max);
    float new_pll_kp = 2.0f * pll_cutoff;
    float new_pll_ki = new_pll_kp * new_pll_kp * 0.25f;

    float omega_motor = smo->Rs * smo->Ls_inv;
    float omega_obs = clampf(TWO_PI * smo->bw_cur, 1.5f * omega_motor, smo->omega_obs_max);
    float R_ratio = omega_obs * smo->Ls - smo->Rs;
    if (R_ratio <= 0.01f) R_ratio = 0.01f;

    float E_est = spd_abs * smo->psi;
    float new_k_slide = 1.5f * E_est + 2.0f;
    float vbus_safe = (vbus > 1.0f) ? vbus : 12.0f;
    float k_slide_max = 2.0f * SQRT3_INV * vbus_safe;
    if (new_k_slide > k_slide_max) {
        new_k_slide = k_slide_max;
    }

    float new_k_sigmoid = new_k_slide / R_ratio;
    if (new_k_sigmoid < 0.01f) {
        new_k_sigmoid = 0.01f;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    smo->omega_est_filt = new_omega_filt;
    smo->pll_kp = new_pll_kp;
    smo->pll_ki = new_pll_ki;
    smo->pll_ki_dt = new_pll_ki * smo->dt;
    smo->k_slide = new_k_slide;
    smo->k_sigmoid = new_k_sigmoid;
    smo->omega_obs = omega_obs;
    if (!primask) {
        __enable_irq();
    }
}

void SMO_Init(SMO_Observer_t* smo) {
    smo->Ialpha_est = 0.0f;
    smo->Ibeta_est = 0.0f;
    smo->Ealpha = 0.0f;
    smo->Ebeta = 0.0f;
    smo->Ealpha_flt = 0.0f;
    smo->Ebeta_flt = 0.0f;
    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;
    smo->omega_out = 0.0f;

    /* Set default gains from config */
    smo->k_slide = SMO_K_SLIDE;
    smo->k_sigmoid = SMO_K_SIGMOID;

    /* PLL gains for angle tracking */
    smo->pll_kp = 2.0f * TWO_PI * SMO_PPL_CUTOFF;   /* 2 * cutoff frequency */
    smo->pll_ki = smo->pll_kp * smo->pll_kp / 4.0f; /* (kp/2)^2 for critical damping */
    smo->pll_integral = 0.0f;
    smo->pll_int_min = SMO_PLL_INT_MIN;
    smo->pll_int_max = SMO_PLL_INT_MAX;

    /* Initialize Dynamic PLL parameters */
    smo->pll_cutoff_min = 10.0f * TWO_PI;
    smo->pll_alpha = 0.5f; /* Speed-to-bandwidth scaling factor */
    smo->omega_est_filt = 0.0f;
    smo->omega_stf = 0.0f;

    /* Initialize 6th Harmonic Compensator parameters */
    smo->Ac = 0.0f;
    smo->As = 0.0f;
    smo->gamma_6th = 20.0f;
    smo->max_comp_norm = 0.02778f; /* +/- 5 degrees normalized */

    smo->current_err_sq = 0.0f;
    smo->theta_err = 0.0f;
    smo->bemf_mag = 0.0f;
    smo->direction = 1.0f;
    smo->omega_obs = 2000.0f;
    smo->phase_lag = 0.0f;
    smo->a_zoh = 0.0f;
    smo->b_zoh = 0.0f;
    smo->Rs_inv = 0.0f;
    smo->tau_ratio_0 = 0.0f;
    smo->dt = CONTROL_PERIOD;
    smo->dt_over_pi = CONTROL_PERIOD * (1.0f / PI);
    smo->dt_1p5_over_pi = 1.5f * smo->dt_over_pi;
    smo->gamma_6th_dt = smo->gamma_6th * CONTROL_PERIOD;
    smo->pll_ki_dt = smo->pll_ki * CONTROL_PERIOD;
    smo->omega_stf_alpha = OMEGA_STF_CUTOFF * CONTROL_PERIOD;
    smo->omega_out_alpha = OMEGA_OUT_CUTOFF * CONTROL_PERIOD;
    smo->pll_cutoff_max = (1.0f / CONTROL_PERIOD) * (TWO_PI / 14.0f);
    smo->omega_obs_max = (1.0f / CONTROL_PERIOD) * (TWO_PI / 6.0f);
}

void SMO_Reset(SMO_Observer_t* smo) {
    smo->Ialpha_est = 0.0f;
    smo->Ibeta_est = 0.0f;
    smo->Ealpha = 0.0f;
    smo->Ebeta = 0.0f;
    smo->Ealpha_flt = 0.0f;
    smo->Ebeta_flt = 0.0f;
    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;
    smo->omega_out = 0.0f;
    smo->omega_stf = 0.0f;
    smo->pll_integral = 0.0f;

    /* Reset adaptive filters and compensators */
    smo->omega_est_filt = 0.0f;
    smo->Ac = 0.0f;
    smo->As = 0.0f;
    smo->current_err_sq = 0.0f;
    smo->theta_err = 0.0f;
    smo->bemf_mag = 0.0f;
    smo->direction = 1.0f;
}

void SMO_ResetStatesWithAngle(SMO_Observer_t* smo, float omega_init, float theta_norm,
                              float Ialpha_init, float Ibeta_init) {
    smo->pll_integral = omega_init;
    smo->omega_est = omega_init;
    smo->omega_out = omega_init;
    smo->omega_stf = omega_init;
    smo->omega_est_filt = fabsf(omega_init);
    smo->theta_est = normalize_angle_norm(theta_norm);
    smo->direction = (omega_init >= 0.0f) ? 1.0f : -1.0f;
    smo->Ac = 0.0f;
    smo->As = 0.0f;

    /* Pre-seed BEMF states corresponding to the handoff angle and speed */
    float cos_th, sin_th;
    cordic_sincos(smo->theta_est, &cos_th, &sin_th);
    float e_mag = omega_init * smo->psi;
    smo->Ealpha = -e_mag * sin_th;
    smo->Ebeta = e_mag * cos_th;
    smo->Ealpha_flt = smo->Ealpha;
    smo->Ebeta_flt = smo->Ebeta;

    /* Seamless current observer handoff: seed current estimates to eliminate
     * initial transient current error spike when returning to FOC_STATE_RUN */
    smo->Ialpha_est = Ialpha_init;
    smo->Ibeta_est = Ibeta_init;
}

void SMO_SeedFlyingStart(SMO_Observer_t* smo, float theta_norm, float omega_elec, float dir,
                         float vbus) {
    float omega_clamped = clampf(omega_elec, smo->pll_int_min, smo->pll_int_max);
    smo->theta_est = normalize_angle_norm(theta_norm);
    smo->omega_est = omega_clamped;
    smo->omega_out = omega_clamped;
    smo->omega_stf = omega_clamped;
    smo->pll_integral = omega_clamped;
    smo->direction = (dir < 0.0f) ? -1.0f : 1.0f;
    SMO_ComputeAndCommitGains(smo, omega_clamped, vbus, 0);
}

void SMO_SeedCurrentFromBEMF(SMO_Observer_t* smo, float Ialpha_init, float Ibeta_init) {
    /* Exact analytical inverse of vector-norm sigmoid:
     * E = k_slide * err / (|err| + k_sigmoid) -> err = E * k_sigmoid / (k_slide - |E|) */
    float scale = 0.0f;
    if (smo->k_slide > 0.1f) {
        float e_sq = smo->Ealpha_flt * smo->Ealpha_flt + smo->Ebeta_flt * smo->Ebeta_flt;
        float e_mag = (e_sq > 0.0f) ? sqrtf(e_sq) : 0.0f;
        float e_max = 0.95f * smo->k_slide;
        if (e_mag > e_max) {
            scale = (19.0f * smo->k_sigmoid) / e_mag;
        } else {
            scale = smo->k_sigmoid / (smo->k_slide - e_mag);
        }
    }

    smo->Ialpha_est = Ialpha_init + smo->Ealpha_flt * scale;
    smo->Ibeta_est = Ibeta_init + smo->Ebeta_flt * scale;
    smo->Ealpha = smo->Ealpha_flt;
    smo->Ebeta = smo->Ebeta_flt;
}

void SMO_SetDirection(SMO_Observer_t* smo, float dir) {
    smo->direction = (dir < 0.0f) ? -1.0f : 1.0f;
}

CCMRAM_FUNC static inline void SMO_PLL_Track(SMO_Observer_t* smo, float sig_gain, float a_zoh_eff,
                                             float b_zoh_eff, uint8_t feed, uint8_t opts) {
    float theta_bemf =
        cordic_atan2(-smo->direction * smo->Ealpha_flt, smo->direction * smo->Ebeta_flt);

    float theta_err = normalize_angle_norm(theta_bemf - smo->theta_est);
    float theta_err_clean = theta_err;
    smo->theta_err = theta_err;

    /* 6th harmonic compensation - full observer path only (flying start feeds
     * clean external BEMF, so its Ac/As state is left untouched). */
    if (!feed) {
        if ((opts & SMO_OPT_HARM_COMP) != 0U) {
            float theta_ref = normalize_angle_norm(6.0f * smo->theta_est);

            float cos_6, sin_6;
            cordic_sincos(theta_ref, &cos_6, &sin_6);

            float theta_comp_6th = smo->Ac * cos_6 + smo->As * sin_6;
            theta_err_clean = normalize_angle_norm(theta_err - theta_comp_6th);

            float gamma_err = smo->gamma_6th_dt * theta_err_clean;
            smo->Ac += gamma_err * cos_6;
            smo->As += gamma_err * sin_6;

            float comp_mag_sq = smo->Ac * smo->Ac + smo->As * smo->As;
            float max_norm_sq = smo->max_comp_norm * smo->max_comp_norm;
            if (comp_mag_sq > max_norm_sq) {
                float inv_scale = smo->max_comp_norm / sqrtf(comp_mag_sq);
                smo->Ac *= inv_scale;
                smo->As *= inv_scale;
            }
        } else {
            smo->Ac *= 0.995f;
            smo->As *= 0.995f;
        }
    }

    float theta_err_rad = theta_err_clean * PI;

    float bemf_sq = smo->Ealpha_flt * smo->Ealpha_flt + smo->Ebeta_flt * smo->Ebeta_flt;

    if (bemf_sq >= 1e-4f) {
        smo->pll_integral += smo->pll_ki_dt * theta_err_rad;
        smo->pll_integral = clampf(smo->pll_integral, smo->pll_int_min, smo->pll_int_max);
    } else {
        smo->pll_integral *= 0.995f;
    }

    float omega_raw = smo->pll_kp * theta_err_rad + smo->pll_integral;

    if ((opts & SMO_OPT_STARTUP) != 0U) {
        if (smo->direction > 0.0f) {
            if (smo->pll_integral < 0.0f) smo->pll_integral = 0.0f;
            if (omega_raw < 0.0f) omega_raw = 0.0f;
        } else {
            if (smo->pll_integral > 0.0f) smo->pll_integral = 0.0f;
            if (omega_raw > 0.0f) omega_raw = 0.0f;
        }
    }

    float omega_limit = smo->pll_int_max * 1.5f;
    if (omega_limit < 1000.0f) omega_limit = 1000.0f;
    smo->omega_est = saturatef(omega_raw, omega_limit);

    smo->omega_out += smo->omega_out_alpha * (smo->omega_est - smo->omega_out);

    smo->theta_est += smo->omega_est * smo->dt_over_pi;
    smo->theta_est = normalize_angle_norm(smo->theta_est);

    if (feed) {
        smo->phase_lag = smo->omega_est * (0.3f * smo->dt_over_pi);
    } else {
        float theta_step_norm = smo->omega_est * smo->dt_over_pi;
        float theta_step = smo->omega_est * smo->dt;
        float th_sq = theta_step * theta_step;
        float cos_step = 1.0f - 0.5f * th_sq + 0.04166667f * th_sq * th_sq;
        float sin_step = theta_step * (1.0f - 0.16666667f * th_sq);
        float z_pole = a_zoh_eff - b_zoh_eff * sig_gain;
        float phi_pole = cordic_atan2(sin_step, cos_step - z_pole);

        smo->phase_lag = phi_pole - 0.50f * theta_step_norm;
    }
}

CCMRAM_FUNC void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha,
                            float Ibeta, float l_app_ratio, uint8_t opts, float ref_omega) {
    /* 1. Extract BEMF via Vector-Norm Sigmoid (circular symmetry + exact K_eff match) */
    float Ialpha_err = smo->Ialpha_est - Ialpha;
    float Ibeta_err = smo->Ibeta_est - Ibeta;
    float err_sq = Ialpha_err * Ialpha_err + Ibeta_err * Ibeta_err;
    smo->current_err_sq = err_sq;

    float I_err_mag = (err_sq > 0.0f) ? sqrtf(err_sq) : 0.0f;
    float sig_gain = smo->k_slide / (I_err_mag + smo->k_sigmoid);
    smo->Ealpha = Ialpha_err * sig_gain;
    smo->Ebeta = Ibeta_err * sig_gain;

    float a_zoh_eff = smo->a_zoh;
    float b_zoh_eff = smo->b_zoh;

    /* Dynamic Magnetic Saturation ZOH update via unconditionally positive Pade-(2,2) half-step
     * squaring (only executed when motor saturation is active and l_app_ratio < 1.0) */
    if (smo->sat_alpha > 0.0f && l_app_ratio < 0.9999f) {
        float h = 0.5f * (smo->tau_ratio_0 / l_app_ratio);
        float q = h * h * 0.08333333f;
        float p = (1.0f - 0.5f * h + q) / (1.0f + 0.5f * h + q);
        a_zoh_eff = p * p;
        b_zoh_eff = (smo->Rs_inv > 0.0f) ? ((1.0f - a_zoh_eff) * smo->Rs_inv) : smo->b_zoh;
    }

    /* Exact Discrete ZOH Current Integration */
    smo->Ialpha_est = a_zoh_eff * smo->Ialpha_est + b_zoh_eff * (Valpha - smo->Ealpha);
    smo->Ibeta_est = a_zoh_eff * smo->Ibeta_est + b_zoh_eff * (Vbeta - smo->Ebeta);

    /* 2. Self-Tuning Filter (STF) for BEMF with capped passband width */
    float target_omega = ((opts & SMO_OPT_STARTUP) != 0U) ? ref_omega : smo->omega_est;
    smo->omega_stf += smo->omega_stf_alpha * (target_omega - smo->omega_stf);
    float wc_max = (smo->pll_cutoff_max > smo->wc_base) ? smo->pll_cutoff_max : smo->wc_base;
    float wc = fminf(smo->wc_base + 0.2f * fabsf(smo->omega_stf), wc_max);

    stf_filter_step(smo->Ealpha, smo->Ebeta, &smo->Ealpha_flt, &smo->Ebeta_flt, wc, smo->omega_stf,
                    smo->dt);

    /* 3. PLL Tracking: pass precomputed sig_gain and live ZOH coefficients into registers */
    SMO_PLL_Track(smo, sig_gain, a_zoh_eff, b_zoh_eff, 0U, opts);
}

static void SMO_RecomputeNominalZOH(SMO_Observer_t* smo) {
    smo->tau_ratio_0 = (smo->Rs * smo->dt) * smo->Ls_inv;
    smo->a_zoh = expf(-smo->tau_ratio_0);
    smo->b_zoh =
        (smo->Rs_inv > 0.0f) ? ((1.0f - smo->a_zoh) * smo->Rs_inv) : (smo->dt * smo->Ls_inv);
}

void SMO_SetMotorParams(SMO_Observer_t* smo, float Rs, float Ls, float sat_alpha,
                        float flux_linkage, float poles, float max_speed_elec_rad, float bw_cur) {
    smo->Rs = Rs;
    smo->Rs_inv = (Rs > 1e-6f) ? (1.0f / Rs) : 0.0f;
    smo->Ls = Ls;
    smo->psi = flux_linkage;
    smo->Ls_inv = (Ls > 1e-7f) ? (1.0f / Ls) : 0.0f;
    smo->sat_alpha = (sat_alpha >= 0.0f) ? sat_alpha : 0.0f;
    smo->poles = poles;

    /* Dynamically calculate PLL integral limits (max electrical speed rad/s) */
    float max_spd = (max_speed_elec_rad > 100.0f) ? max_speed_elec_rad : 5000.0f;
    smo->pll_int_max = max_spd;
    smo->pll_int_min = -max_spd;

    /* Update Exact Discrete ZOH coefficients */
    SMO_RecomputeNominalZOH(smo);
    smo->bw_cur = bw_cur;
    float f_bw_min = clampf(0.5f * (smo->Rs * smo->Ls_inv * (1.0f / TWO_PI)), 20.0f, 200.0f);
    smo->wc_base = TWO_PI * f_bw_min;

    /* Initialize nominal gains when motor is stationary so startup/flying-start never start with
     * hardcoded defaults before the first 1kHz SlowTask tick */
    if (fabsf(smo->omega_est) < 1.0f && smo->omega_est_filt < 1.0f && smo->Ls > 1e-7f &&
        smo->bw_cur > 0.0f) {
        SMO_ComputeAndCommitGains(smo, 0.0f, 12.0f, 0U);
    }
}

void SMO_SetTiming(SMO_Observer_t* smo, float dt) {
    if (dt > 0.0f) {
        smo->dt = dt;
        SMO_RecomputeNominalZOH(smo);
        smo->dt_over_pi = smo->dt * (1.0f / PI);
        smo->dt_1p5_over_pi = 1.5f * smo->dt_over_pi;
        smo->gamma_6th_dt = smo->gamma_6th * smo->dt;
        smo->pll_ki_dt = smo->pll_ki * smo->dt;

        float fs = 1.0f / smo->dt;
        smo->omega_stf_alpha = OMEGA_STF_CUTOFF * smo->dt;
        smo->omega_out_alpha = OMEGA_OUT_CUTOFF * smo->dt;
        smo->pll_cutoff_max = fs * (TWO_PI / 14.0f);
        smo->omega_obs_max = fs * (TWO_PI / 6.0f);
    }
}

void SMO_FeedBEMF(SMO_Observer_t* smo, float Ealpha, float Ebeta) {
    smo->omega_stf += smo->omega_stf_alpha * (smo->omega_est - smo->omega_stf);
    float wc_max = (smo->pll_cutoff_max > smo->wc_base) ? smo->pll_cutoff_max : smo->wc_base;
    float wc = clampf(smo->wc_base + 0.5f * fabsf(smo->omega_stf), smo->wc_base, wc_max);
    stf_filter_step(Ealpha, Ebeta, &smo->Ealpha_flt, &smo->Ebeta_flt, wc, smo->omega_stf, smo->dt);

    SMO_PLL_Track(smo, 0.0f, smo->a_zoh, smo->b_zoh, 1U, SMO_OPT_NONE);
}

void SMO_SlowTask(SMO_Observer_t* smo, uint8_t is_active, uint8_t is_startup, float startup_omega,
                  float vbus) {
    if (!is_active) {
        smo->bemf_mag = 0.0f;
        return;
    }

    float spd_ref = is_startup ? startup_omega : smo->omega_est;
    SMO_ComputeAndCommitGains(smo, spd_ref, vbus, 1U);

    /* Compute Back-EMF magnitude at 1kHz rate for diagnostics & flux adaptation (deferred from
     * 48kHz ISR) */
    float bemf_sq = smo->Ealpha_flt * smo->Ealpha_flt + smo->Ebeta_flt * smo->Ebeta_flt;
    smo->bemf_mag = (bemf_sq > 0.0f) ? sqrtf(bemf_sq) : 0.0f;
}