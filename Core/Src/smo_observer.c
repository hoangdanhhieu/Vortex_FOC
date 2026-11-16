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
#include "foc_state_machine.h"

static float omega_stf_alpha;
static float omega_out_alpha;
static float dt_over_pi;
static float pll_cutoff_max;
static float omega_obs_max;

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
    smo->enable_harmonic_comp = 0;

    smo->I_mag = 0.0f;
    smo->l_ratio = 1.0f;
    smo->current_err_sq = 0.0f;
    smo->theta_err = 0.0f;
    smo->bemf_mag = 0.0f;
    smo->direction = 1.0f;
    smo->omega_obs = 2000.0f;
    omega_obs_max = 1.0f;
    smo->phase_lag = 0.0f;
    smo->a_zoh = 0.0f;
    smo->b_zoh = 0.0f;
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
    smo->I_mag = 0.0f;
    smo->l_ratio = 1.0f;
    smo->enable_harmonic_comp = 0;
    smo->direction = 1.0f;
}

void SMO_ResetStates(SMO_Observer_t* smo, float omega_init) {
    smo->pll_integral = omega_init;
    smo->omega_est = omega_init;
    smo->omega_out = omega_init;
    smo->omega_stf = omega_init;
    smo->omega_est_filt = fabsf(omega_init);
    smo->direction = (omega_init >= 0.0f) ? 1.0f : -1.0f;
    smo->Ac = 0.0f;
    smo->As = 0.0f;
}

void SMO_ResetStatesWithAngle(SMO_Observer_t* smo, float omega_init, float theta_norm) {
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
    smo->Ialpha_est = g_foc.data.Ialpha_flt;
    smo->Ibeta_est = g_foc.data.Ibeta_flt;
}

void SMO_SetDirection(SMO_Observer_t* smo, float dir) {
    smo->direction = (dir < 0.0f) ? -1.0f : 1.0f;
}

CCMRAM_FUNC static inline void SMO_PLL_Track(SMO_Observer_t* smo) {
    const uint8_t feed = (g_foc.status.state == FOC_STATE_FLYING_START);

    /* Direction (+1.0f or -1.0f) is governed exclusively by FOC State Machine.
     * Eliminates positive-feedback discontinuity near zero speed. */
    float theta_bemf =
        cordic_atan2(-smo->direction * smo->Ealpha_flt, smo->direction * smo->Ebeta_flt);

    float theta_comp = theta_bemf;
    float theta_err = normalize_angle_norm(theta_comp - smo->theta_est);
    float theta_err_clean = theta_err;
    smo->theta_err = theta_err;

    /* 6th harmonic compensation - full observer path only (flying start feeds
     * clean external BEMF, so its Ac/As state is left untouched). */
    if (!feed) {
        if (smo->enable_harmonic_comp) {
            float theta_ref = normalize_angle_norm(6.0f * smo->theta_est);

            float cos_6, sin_6;
            cordic_sincos(theta_ref, &cos_6, &sin_6);

            float theta_comp_6th = smo->Ac * cos_6 + smo->As * sin_6;
            theta_err_clean = normalize_angle_norm(theta_err - theta_comp_6th);

            smo->Ac += smo->gamma_6th_dt * theta_err_clean * cos_6;
            smo->As += smo->gamma_6th_dt * theta_err_clean * sin_6;

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
    smo->bemf_mag = (bemf_sq > 0.0f) ? sqrtf(bemf_sq) : 0.0f;

    if (bemf_sq >= 1e-4f) {
        smo->pll_integral += smo->pll_ki_dt * theta_err_rad;
        if (g_foc.status.state == FOC_STATE_STARTUP) {
            if (smo->direction > 0.0f) {
                if (smo->pll_integral < 0.0f) smo->pll_integral = 0.0f;
            } else {
                if (smo->pll_integral > 0.0f) smo->pll_integral = 0.0f;
            }
        }
        smo->pll_integral = clampf(smo->pll_integral, smo->pll_int_min, smo->pll_int_max);
    } else {
        smo->pll_integral *= 0.995f;
    }

    float omega_raw = smo->pll_kp * theta_err_rad + smo->pll_integral;
    if (g_foc.status.state == FOC_STATE_STARTUP) {
        if (smo->direction > 0.0f) {
            if (omega_raw < 0.0f) omega_raw = 0.0f;
        } else {
            if (omega_raw > 0.0f) omega_raw = 0.0f;
        }
    }

    float omega_limit = smo->pll_int_max * 1.5f;
    if (omega_limit < 1000.0f) omega_limit = 1000.0f;
    smo->omega_est = saturatef(omega_raw, omega_limit);

    smo->omega_out += omega_out_alpha * (smo->omega_est - smo->omega_out);

    smo->theta_est += smo->omega_est * smo->dt_over_pi;
    smo->theta_est = normalize_angle_norm(smo->theta_est);

    if (feed) {
        smo->phase_lag = smo->omega_est * (0.3f * smo->dt_over_pi);
    } else {
        float theta_step_norm = smo->omega_est * smo->dt_over_pi;
        float cos_step, sin_step;
        cordic_sincos(theta_step_norm, &cos_step, &sin_step);

        /* Exact discrete ZOH pole phase lag matching vector-norm sigmoid gain */
        float I_err_mag = (smo->current_err_sq > 0.0f) ? sqrtf(smo->current_err_sq) : 0.0f;
        float K_eff = smo->k_slide / (I_err_mag + smo->k_sigmoid);
        float z_pole = smo->a_zoh - smo->b_zoh * K_eff;
        float phi_pole = cordic_atan2(sin_step, cos_step - z_pole);

        /* Net timing correction: balances Type-2 PLL 1.0 dt forward step advance
         * against 0.50 dt continuous stator BEMF midpoint integration lag. */
        smo->phase_lag = phi_pole - 0.50f * theta_step_norm;
    }
}

CCMRAM_FUNC void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha,
                            float Ibeta) {
    /* 0. Measure total current magnitude */
    float I_sq = Ialpha * Ialpha + Ibeta * Ibeta;

    /* 1. Extract BEMF via Vector-Norm Sigmoid (circular symmetry + exact K_eff match) */
    float Ialpha_err = smo->Ialpha_est - Ialpha;
    float Ibeta_err = smo->Ibeta_est - Ibeta;
    smo->current_err_sq = Ialpha_err * Ialpha_err + Ibeta_err * Ibeta_err;

    float I_err_mag = (smo->current_err_sq > 0.0f) ? sqrtf(smo->current_err_sq) : 0.0f;
    float sig_gain = smo->k_slide / (I_err_mag + smo->k_sigmoid);
    smo->Ealpha = Ialpha_err * sig_gain;
    smo->Ebeta = Ibeta_err * sig_gain;

    /* 2. Compute saturation ratio for external PI gain scheduling (do NOT modify SMO internals) */
    if (smo->sat_alpha > 0.0f) {
        float sat_scale = 1.0f + smo->sat_alpha * I_sq;
        smo->l_ratio = clampf(1.0f / sat_scale, 0.20f, 1.0f);
    } else {
        smo->l_ratio = 1.0f;
    }

    /* Exact Discrete ZOH Current Integration */
    smo->Ialpha_est = smo->a_zoh * smo->Ialpha_est + smo->b_zoh * (Valpha - smo->Ealpha);
    smo->Ibeta_est = smo->a_zoh * smo->Ibeta_est + smo->b_zoh * (Vbeta - smo->Ebeta);

    /* 3. Self-Tuning Filter (STF) for BEMF with capped passband width */
    float target_omega =
        (g_foc.status.state == FOC_STATE_STARTUP) ? g_foc.startup.omega : smo->omega_est;
    smo->omega_stf += omega_stf_alpha * (target_omega - smo->omega_stf);
    float wc_max = (pll_cutoff_max > smo->wc_base) ? pll_cutoff_max : smo->wc_base;
    float wc = clampf(smo->wc_base + 0.2f * fabsf(smo->omega_stf), smo->wc_base, wc_max);

    stf_filter_step(smo->Ealpha, smo->Ebeta, &smo->Ealpha_flt, &smo->Ebeta_flt, wc, smo->omega_stf,
                    smo->dt);

    /* 4. PLL Tracking */
    SMO_PLL_Track(smo);
}

void SMO_SetMotorParams(SMO_Observer_t* smo, float Rs, float Ls, float sat_alpha,
                        float flux_linkage, float poles, float max_speed_elec_rad, float bw_cur) {
    smo->Rs = Rs;
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
    float tau_ratio = (smo->Rs * smo->dt) * smo->Ls_inv;
    smo->a_zoh = expf(-tau_ratio);
    smo->b_zoh = (1.0f - smo->a_zoh) / smo->Rs;
    smo->bw_cur = bw_cur;
    float f_bw_min = clampf(0.5f * (smo->Rs * smo->Ls_inv * (1.0f / TWO_PI)), 20.0f, 200.0f);
    smo->wc_base = TWO_PI * f_bw_min;
}

void SMO_SetTiming(SMO_Observer_t* smo, float dt) {
    if (dt > 0.0f) {
        smo->dt = dt;
        float tau_ratio = (smo->Rs * smo->dt) * smo->Ls_inv;
        smo->a_zoh = expf(-tau_ratio);
        smo->b_zoh = (1.0f - smo->a_zoh) / smo->Rs;
        smo->dt_over_pi = smo->dt * (1.0f / PI);
        smo->gamma_6th_dt = smo->gamma_6th * smo->dt;
        smo->pll_ki_dt = smo->pll_ki * smo->dt;

        omega_stf_alpha = (OMEGA_STF_CUTOFF * smo->dt);
        omega_out_alpha = (OMEGA_OUT_CUTOFF * smo->dt);
        pll_cutoff_max = g_foc.cfg.pwm_frequency / 14.0f * TWO_PI;
        omega_obs_max = TWO_PI * (g_foc.cfg.pwm_frequency / 6.0f);
        dt_over_pi = smo->dt_over_pi;
    }
}

void SMO_FeedBEMF(SMO_Observer_t* smo, float Ealpha, float Ebeta) {
    smo->omega_stf += omega_stf_alpha * (smo->omega_est - smo->omega_stf);
    float wc_max = (pll_cutoff_max > smo->wc_base) ? pll_cutoff_max : smo->wc_base;
    float wc = clampf(smo->wc_base + 0.5f * fabsf(smo->omega_stf), smo->wc_base, wc_max);
    stf_filter_step(Ealpha, Ebeta, &smo->Ealpha_flt, &smo->Ebeta_flt, wc, smo->omega_stf, smo->dt);

    SMO_PLL_Track(smo);
}

void SMO_SlowTask(SMO_Observer_t* smo) {
    /* 1. Dynamic PLL Cutoff Frequency based on filtered speed */
    float spd_ref =
        (g_foc.status.state == FOC_STATE_STARTUP) ? g_foc.startup.omega : smo->omega_est;
    if (smo->omega_est_filt == 0.0f && fabsf(spd_ref) > 1.0f) {
        smo->omega_est_filt = fabsf(spd_ref);
    } else {
        smo->omega_est_filt += 0.2f * (fabsf(spd_ref) - smo->omega_est_filt);
    }

    float f_elec = smo->omega_est_filt;
    float pll_cutoff = clampf(smo->pll_alpha * f_elec, smo->pll_cutoff_min, pll_cutoff_max);

    float new_pll_kp = 2.0f * pll_cutoff;
    float new_pll_ki = new_pll_kp * new_pll_kp / 4.0f;

    float omega_motor = smo->Rs * smo->Ls_inv;
    float omega_obs = clampf(TWO_PI * smo->bw_cur, 1.5f * omega_motor, omega_obs_max);
    float R_ratio = omega_obs * smo->Ls - smo->Rs;
    if (R_ratio <= 0.01f) R_ratio = 0.01f;

    float omega_e = fabsf(spd_ref);
    float E_est = omega_e * smo->psi;
    float new_k_slide = 1.5f * E_est + 2.0f;

    float k_slide_max = 2.0f * SQRT3_INV * g_foc.data.Vbus;
    if (new_k_slide > k_slide_max) {
        new_k_slide = k_slide_max;
    }

    float new_k_sigmoid = new_k_slide / R_ratio;
    float k_sig_min = 0.01f;
    if (new_k_sigmoid < k_sig_min) {
        new_k_sigmoid = k_sig_min;
    }
    float delta_v = new_k_slide - E_est;
    if (delta_v < 0.15f * new_k_slide) {
        delta_v = 0.15f * new_k_slide;
    }

    float z_target = expf(-omega_obs * smo->dt);
    float tau_ratio = (smo->Rs * smo->dt) * smo->Ls_inv;
    float new_a_zoh = expf(-tau_ratio);
    float new_b_zoh = (1.0f - new_a_zoh) / smo->Rs;
    float K_target = (new_b_zoh > 1e-9f) ? ((new_a_zoh - z_target) / new_b_zoh) : (0.5f * smo->Rs);
    if (K_target < 0.05f * smo->Rs) {
        K_target = 0.05f * smo->Rs;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    smo->pll_kp = new_pll_kp;
    smo->pll_ki = new_pll_ki;
    smo->pll_ki_dt = new_pll_ki * smo->dt;
    smo->k_slide = new_k_slide;
    smo->k_sigmoid = new_k_sigmoid;
    smo->a_zoh = new_a_zoh;
    smo->b_zoh = new_b_zoh;
    smo->omega_obs = omega_obs;
    if (!primask) {
        __enable_irq();
    }
}