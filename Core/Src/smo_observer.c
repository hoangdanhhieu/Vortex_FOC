/**
 * @file smo_observer.c
 * @brief Sliding Mode Observer implementation
 *
 * This SMO uses a continuous sigmoid function instead of sign()
 * to reduce chattering while maintaining fast response.
 */

#include "smo_observer.h"

#include <math.h>

#include "cordic_math.h"
#include "foc.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "motor_params.h"

static float omega_stf_alpha;
static float omega_out_alpha;
static float dt_over_pi;
/**
 * @brief Sigmoid switching function (continuous approximation of sign)
 * @param x Input value
 * @param k Bandwidth parameter (larger = sharper transition)
 * @return Output in range (-1, 1)
 */
CCMRAM_FUNC static inline float sigmoid(float x, float k) {
    /* Fast sigmoid: x / (|x| + k) */
    float abs_x = fabsf(x);
    return x / (abs_x + k);
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
    smo->pll_cutoff_min = 10.0f;
    smo->pll_alpha = 0.5f; /* Speed-to-bandwidth scaling factor */
    smo->omega_est_filt = 0.0f;
    smo->omega_stf = 0.0f;

    /* Initialize 6th Harmonic Compensator parameters */
    smo->Ac = 0.0f;
    smo->As = 0.0f;
    smo->gamma_6th = 20.0f;
    smo->max_comp_norm = 0.02778f; /* +/- 5 degrees normalized */
    smo->enable_harmonic_comp = 0;

    /* Cache motor parameters */
    smo->Rs = MOTOR_RS;
    smo->Ls = MOTOR_LS;
    smo->psi = MOTOR_FLUX_LINKAGE;
    smo->Ls_inv = 1.0f / MOTOR_LS;
    smo->sat_alpha = MOTOR_ALPHA;

    /* Sample time */
    smo->dt = (g_foc.dt > 0.0f) ? g_foc.dt : CONTROL_PERIOD;

    /* Pre-calculated current observer time constant and coefficients */
    smo->dt_over_Ls = smo->dt * smo->Ls_inv;
    smo->denom_inv = 1.0f / (1.0f + smo->Rs * smo->dt_over_Ls);
    smo->I_mag = 0.0f;
    smo->l_ratio = 1.0f;

    smo->dt_over_pi = smo->dt * (1.0f / PI);
    smo->gamma_6th_dt = smo->gamma_6th * smo->dt;
    smo->pll_ki_dt = smo->pll_ki * smo->dt;

    float f_bw_min = clampf(0.5f * (smo->Rs * smo->Ls_inv * (1.0f / TWO_PI)), 20.0f, 200.0f);
    smo->wc_base = TWO_PI * f_bw_min;

    omega_stf_alpha = (TWO_PI * 800.0f * smo->dt);
    omega_out_alpha = (TWO_PI * 300.0f * smo->dt);
    dt_over_pi = smo->dt_over_pi;
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
}

void SMO_ResetStates(SMO_Observer_t* smo, float omega_init) {
    smo->pll_integral = omega_init;
    smo->omega_est = omega_init;
    smo->omega_out = omega_init;
    smo->omega_stf = omega_init;
}

CCMRAM_FUNC static inline void SMO_PLL_Track(SMO_Observer_t* smo) {
    float theta_bemf = cordic_atan2(-smo->Ealpha_flt, smo->Ebeta_flt);

    /* Internal Observer Phase Delay Compensation (Total = 1.0 dt):
     * - 0.5 dt from Backward Euler integration (midpoint lag).
     * - 0.5 dt from Voltage Mismatch: The ADC samples exactly at the Peak (t=0).
     *   The actual voltage applied to the motor between two samples is V[N-2] for
     *   the first 0.5 dt, and V[N-1] for the second 0.5 dt. However, the observer
     *   uses V[N-1] for the entire 1.0 dt. This "pulls" the voltage forward in time,
     *   causing a 0.5 dt lag in the resulting BEMF estimate.
     */
    float tau_total = (1.0f + g_foc.cfg.comp_delay_samples) * smo->dt;
    float x = smo->omega_est * tau_total;
    float x_sq = x * x;
    float phase_lag = x * (1.0f - x_sq * (0.33333333f - 0.2f * x_sq)) * (1.0f / PI);

    float theta_comp = theta_bemf + phase_lag;
    float theta_err = normalize_angle_norm(theta_comp - smo->theta_est);
    float theta_err_clean = theta_err;

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

    float theta_err_rad = theta_err_clean * PI;

    float bemf_sq = smo->Ealpha_flt * smo->Ealpha_flt + smo->Ebeta_flt * smo->Ebeta_flt;
    smo->bemf_mag = (bemf_sq > 0.0f) ? sqrtf(bemf_sq) : 0.0f;

    if (bemf_sq >= 1e-4f) {
        smo->pll_integral += smo->pll_ki_dt * theta_err_rad;
        if (FOC_GetState() == FOC_STATE_STARTUP) {
            if (smo->pll_integral < 0.0f) smo->pll_integral = 0.0f;
        }
        smo->pll_integral = clampf(smo->pll_integral, smo->pll_int_min, smo->pll_int_max);
    } else {
        smo->pll_integral *= 0.995f;
    }

    float omega_raw = smo->pll_kp * theta_err_rad + smo->pll_integral;
    if (FOC_GetState() == FOC_STATE_STARTUP && omega_raw < 0.0f) {
        omega_raw = 0.0f;
    }

    float omega_limit = smo->pll_int_max * 1.5f;
    if (omega_limit < 1000.0f) omega_limit = 1000.0f;
    smo->omega_est = saturatef(omega_raw, omega_limit);

    smo->omega_out += omega_out_alpha * (smo->omega_est - smo->omega_out);

    smo->theta_est += smo->omega_est * smo->dt_over_pi;
    smo->theta_est = normalize_angle_norm(smo->theta_est);
}

CCMRAM_FUNC void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha,
                            float Ibeta) {
    /* 0. Measure total current magnitude */
    float I_sq = Ialpha * Ialpha + Ibeta * Ibeta;
    // smo->I_mag = sqrtf(I_sq);

    /* 1. Extract BEMF from observer state (Standard high-SNR Sigmoid SMO without attenuation) */
    float Ialpha_err = smo->Ialpha_est - Ialpha;
    float Ibeta_err = smo->Ibeta_est - Ibeta;
    smo->current_err_sq = Ialpha_err * Ialpha_err + Ibeta_err * Ibeta_err;

    smo->Ealpha = sigmoid(Ialpha_err, smo->k_sigmoid) * smo->k_slide;
    smo->Ebeta = sigmoid(Ibeta_err, smo->k_sigmoid) * smo->k_slide;

    /* 2. Compute saturation ratio for external PI gain scheduling (do NOT modify SMO internals) */
    if (smo->sat_alpha > 0.0f) {
        float sat_scale = 1.0f + smo->sat_alpha * I_sq;
        smo->l_ratio = clampf(1.0f / sat_scale, 0.20f, 1.0f);
    } else {
        smo->l_ratio = 1.0f;
    }

    /* Backward Euler Current Integration (using nominal L0 — observer gains are tuned for this) */
    smo->Ialpha_est = (smo->Ialpha_est + (Valpha - smo->Ealpha) * smo->dt_over_Ls) * smo->denom_inv;
    smo->Ibeta_est = (smo->Ibeta_est + (Vbeta - smo->Ebeta) * smo->dt_over_Ls) * smo->denom_inv;

    /* 3. Self-Tuning Filter (STF) for BEMF using Backward Euler with smooth frequency tracking */
    float target_omega =
        (FOC_GetState() == FOC_STATE_STARTUP) ? g_foc.startup.omega : smo->omega_est;
    smo->omega_stf += omega_stf_alpha * (target_omega - smo->omega_stf);
    float omega_stf = smo->omega_stf;
    float wc = smo->wc_base + 0.8f * fabsf(omega_stf);

    stf_filter_step(smo->Ealpha, smo->Ebeta, &smo->Ealpha_flt, &smo->Ebeta_flt, wc, omega_stf,
                    smo->dt);

    /* 4. PLL Tracking */
    SMO_PLL_Track(smo);
}

void SMO_SetMotorParams(SMO_Observer_t* smo, float Rs, float Ls, float sat_alpha,
                        float flux_linkage, float poles, float max_speed_elec_rad) {
    smo->Rs = Rs;
    smo->Ls = Ls;
    smo->psi = flux_linkage;
    smo->Ls_inv = 1.0f / Ls;
    smo->sat_alpha = (sat_alpha >= 0.0f) ? sat_alpha : 0.0f;
    smo->poles = poles;

    /* Dynamically calculate PLL integral limits (max electrical speed rad/s) */
    smo->pll_int_max = max_speed_elec_rad;
    smo->pll_int_min = -max_speed_elec_rad;

    /* Update Euler coefficients */
    smo->dt_over_Ls = smo->dt * smo->Ls_inv;
    smo->denom_inv = 1.0f / (1.0f + smo->Rs * smo->dt_over_Ls);

    float f_bw_min = clampf(0.5f * (smo->Rs * smo->Ls_inv * (1.0f / TWO_PI)), 20.0f, 200.0f);
    smo->wc_base = TWO_PI * f_bw_min;
}

void SMO_SetTiming(SMO_Observer_t* smo, float dt) {
    if (dt > 0.0f) {
        smo->dt = dt;
        smo->dt_over_Ls = smo->dt * smo->Ls_inv;
        smo->denom_inv = 1.0f / (1.0f + smo->Rs * smo->dt_over_Ls);
        smo->dt_over_pi = smo->dt * (1.0f / PI);
        smo->gamma_6th_dt = smo->gamma_6th * smo->dt;
        smo->pll_ki_dt = smo->pll_ki * smo->dt;

        omega_stf_alpha = (TWO_PI * 500.0f * smo->dt);
        omega_out_alpha = (TWO_PI * 300.0f * smo->dt);
        dt_over_pi = smo->dt_over_pi;
    }
}

void SMO_FeedBEMF(SMO_Observer_t* smo, float Ealpha, float Ebeta) {
    /* Self-Tuning Filter (STF) for BEMF using Backward Euler (unconditionally stable) */
    smo->omega_stf += omega_stf_alpha * (smo->omega_est - smo->omega_stf);
    float omega_stf = smo->omega_stf;
    float wc = smo->wc_base + 0.8f * fabsf(omega_stf);

    float a = wc * smo->dt;
    float b = omega_stf * smo->dt;
    float D_inv = 1.0f / ((1.0f + a) * (1.0f + a) + b * b);

    float r_alpha = smo->Ealpha_flt + a * Ealpha;
    float r_beta = smo->Ebeta_flt + a * Ebeta;

    smo->Ealpha_flt = ((1.0f + a) * r_alpha - b * r_beta) * D_inv;
    smo->Ebeta_flt = (b * r_alpha + (1.0f + a) * r_beta) * D_inv;

    float theta_bemf = cordic_atan2(-smo->Ealpha_flt, smo->Ebeta_flt);

    /* Hardware ADC Regular Sampling Delay Compensation (0.3 dt) */
    float tau_adc = 0.3f * smo->dt;
    float x = smo->omega_est * tau_adc;
    float x_sq = x * x;
    float phase_lag = x * (1.0f - x_sq * (0.33333333f - 0.2f * x_sq)) * (1.0f / PI);

    float theta_comp = theta_bemf + phase_lag;
    float theta_err = normalize_angle_norm(theta_comp - smo->theta_est);
    float theta_err_rad = theta_err * PI;

    smo->theta_err = theta_err;

    float bemf_sq = smo->Ealpha_flt * smo->Ealpha_flt + smo->Ebeta_flt * smo->Ebeta_flt;
    smo->bemf_mag = (bemf_sq > 0.0f) ? sqrtf(bemf_sq) : 0.0f;

    if (bemf_sq >= 1e-4f) {
        smo->pll_integral += smo->pll_ki_dt * theta_err_rad;
        smo->pll_integral = clampf(smo->pll_integral, smo->pll_int_min, smo->pll_int_max);
    } else {
        smo->pll_integral *= 0.995f;
    }

    float omega_raw = smo->pll_kp * theta_err_rad + smo->pll_integral;
    float omega_limit = smo->pll_int_max * 1.5f;
    if (omega_limit < 1000.0f) omega_limit = 1000.0f;
    smo->omega_est = saturatef(omega_raw, omega_limit);

    smo->omega_out += omega_out_alpha * (smo->omega_est - smo->omega_out);

    smo->theta_est += smo->omega_est * smo->dt_over_pi;
    smo->theta_est = normalize_angle_norm(smo->theta_est);
}

void SMO_SlowTask(SMO_Observer_t* smo) {
    /* 1. Dynamic PLL Cutoff Frequency based on filtered speed */
    float spd_ref = (FOC_GetState() == FOC_STATE_STARTUP) ? g_foc.startup.omega : smo->omega_est;
    if (smo->omega_est_filt == 0.0f && fabsf(spd_ref) > 1.0f) {
        smo->omega_est_filt = fabsf(spd_ref);
    } else {
        smo->omega_est_filt += 0.2f * (fabsf(spd_ref) - smo->omega_est_filt);
    }

    float f_elec = smo->omega_est_filt / TWO_PI;
    float pll_cutoff_hz = smo->pll_alpha * f_elec;
    if (pll_cutoff_hz < smo->pll_cutoff_min) pll_cutoff_hz = smo->pll_cutoff_min;
    if (pll_cutoff_hz > 2500.0f) pll_cutoff_hz = 2500.0f;

    float new_pll_kp = 2.0f * TWO_PI * pll_cutoff_hz;
    float new_pll_ki = new_pll_kp * new_pll_kp / 4.0f;

    float omega_motor = smo->Rs * smo->Ls_inv;
    float omega_obs = 2.0f * PI * g_foc.cfg.bw_cur;
    if (omega_obs < 1.5f * omega_motor) {
        omega_obs = 1.5f * omega_motor;
    }
    float R_ratio = omega_obs * smo->Ls - smo->Rs;
    if (R_ratio <= 0.01f) R_ratio = 0.01f;

    float omega_e = fabsf(spd_ref);
    float E_est = omega_e * smo->psi;
    float new_k_slide = 1.5f * E_est + 2.0f;

    float k_slide_max = 15.0f * (smo->Ls / smo->dt);
    float safety_net = 2.0f * E_est + 5.0f;
    if (k_slide_max < safety_net) {
        k_slide_max = safety_net;
    }
    if (new_k_slide > k_slide_max) {
        new_k_slide = k_slide_max;
    }

    float new_k_sigmoid = new_k_slide / R_ratio;
    float k_sig_min = 0.001f;
    if (new_k_sigmoid < k_sig_min) {
        new_k_sigmoid = k_sig_min;
    }

    float new_denom_inv = 1.0f / (1.0f + smo->Rs * smo->dt_over_Ls);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    smo->pll_kp = new_pll_kp;
    smo->pll_ki = new_pll_ki;
    smo->pll_ki_dt = new_pll_ki * smo->dt;
    smo->k_slide = new_k_slide;
    smo->k_sigmoid = new_k_sigmoid;
    smo->denom_inv = new_denom_inv;
    if (!primask) {
        __enable_irq();
    }
}
