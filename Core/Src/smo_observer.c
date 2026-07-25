/**
 * @file smo_observer.c
 * @brief Sliding Mode Observer implementation
 *
 * This SMO uses a continuous sigmoid function instead of sign()
 * to reduce chattering while maintaining fast response.
 */

#include "smo_observer.h"

#include "cordic_math.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "math.h"
#include "motor_params.h"
/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Private Functions                                                         */
/*===========================================================================*/

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

/**
 * @brief Normalize angle to [-1, 1) range (corresponds to [-pi, pi))
 */
CCMRAM_FUNC static inline float normalize_angle(float angle) {
    return angle - 2.0f * floorf((angle + 1.0f) * 0.5f);
}

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

void SMO_Init(SMO_Observer_t* smo) {
    /* Clear all states */
    smo->Ialpha_est = 0.0f;
    smo->Ibeta_est = 0.0f;
    smo->Ealpha = 0.0f;
    smo->Ebeta = 0.0f;
    smo->Ealpha_flt = 0.0f;
    smo->Ebeta_flt = 0.0f;
    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;

    /* Set default gains from config */
    smo->k_slide = SMO_K_SLIDE;
    smo->k_sigmoid = SMO_K_SIGMOID;

    /* Initial LPF coefficient (dynamically updated in SMO_Update based on speed) */
    smo->tau = 1.0f / 60.0f;
    smo->lpf_coeff = CONTROL_PERIOD / (CONTROL_PERIOD + smo->tau);

    /* PLL gains for angle tracking */
    smo->pll_kp = 2.0f * TWO_PI * SMO_PPL_CUTOFF;   /* 2 * cutoff frequency */
    smo->pll_ki = smo->pll_kp * smo->pll_kp / 4.0f; /* (kp/2)^2 for critical damping */
    smo->pll_integral = 0.0f;
    smo->pll_int_min = SMO_PLL_INT_MIN;
    smo->pll_int_max = SMO_PLL_INT_MAX;

    /* Initialize Dynamic PLL parameters */
    smo->pll_cutoff_min = 8.0f; /* 50 Hz minimum bandwidth */
    smo->pll_alpha = 0.5f;      /* Speed-to-bandwidth scaling factor */
    smo->omega_est_filt = 0.0f;
    smo->omega_stf = 0.0f;

    /* Initialize 6th Harmonic Compensator parameters */
    smo->Ac = 0.0f;
    smo->As = 0.0f;
    smo->gamma_6th = 20.0f;
    smo->max_comp_norm = 0.02778f; /* +/- 5 degrees normalized */
    smo->err_dc = 0.0f;
    smo->enable_harmonic_comp = 0;

    /* Cache motor parameters */
    smo->Rs = MOTOR_RS;
    smo->Ls = MOTOR_LS;
    smo->psi = MOTOR_FLUX_LINKAGE;
    smo->Ls_inv = 1.0f / MOTOR_LS;

    /* Sample time */
    smo->dt = CONTROL_PERIOD;

    /* Pre-calculated current observer time constant */
    smo->tau_current = MOTOR_LS / MOTOR_RS;
}

CCMRAM_FUNC void SMO_Reset(SMO_Observer_t* smo) {
    smo->Ialpha_est = 0.0f;
    smo->Ibeta_est = 0.0f;
    smo->Ealpha = 0.0f;
    smo->Ebeta = 0.0f;
    smo->Ealpha_flt = 0.0f;
    smo->Ebeta_flt = 0.0f;
    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;
    smo->omega_stf = 0.0f;
    smo->pll_integral = 0.0f;

    /* Reset adaptive filters and compensators */
    smo->omega_est_filt = 0.0f;
    smo->Ac = 0.0f;
    smo->As = 0.0f;
    smo->err_dc = 0.0f;
    smo->current_err_sq = 0.0f;
}

CCMRAM_FUNC static inline void SMO_PLL_Track(SMO_Observer_t* smo) {
    float theta_bemf = cordic_atan2(-smo->Ealpha_flt, smo->Ebeta_flt);
    /* Total Phase Lag = Current observer lag + 0.5-sample discrete Backward Euler lag (STF at
     * center frequency has 0 phase lag) */
    float tau_total = smo->tau_current + 0.5f * smo->dt;
    float phase_lag = cordic_atan2(smo->omega_est * tau_total, 1.0f);

    float theta_comp = normalize_angle(theta_bemf + phase_lag);
    float theta_err = normalize_angle(theta_comp - smo->theta_est);
    float theta_err_clean = theta_err;

    /* 6th Harmonic Adaptive Compensation */
    if (smo->enable_harmonic_comp) {
        /* Correct phase delay calculation: exactly 6 times fundamental phase lag */
        /* This avoids calling CORDIC atan2 again, saving CPU cycles */
        float phase_lag_6th = 6.0f * phase_lag;

        /* Generate phase-corrected reference angle in [-1, 1) range */
        float theta_ref = normalize_angle(6.0f * smo->theta_est - phase_lag_6th);

        float cos_6, sin_6;
        cordic_sincos(theta_ref, &cos_6, &sin_6);

        /* Subtraction of estimated ripple (Ac, As are in normalized angle units) */
        float theta_comp_6th = smo->Ac * cos_6 + smo->As * sin_6;
        theta_err_clean = normalize_angle(theta_err - theta_comp_6th);

        /* LMS adaptive updates using theta_err_clean directly */
        /* (The correlation with cos/sin naturally rejects DC components) */
        smo->Ac += smo->gamma_6th * theta_err_clean * cos_6 * smo->dt;
        smo->As += smo->gamma_6th * theta_err_clean * sin_6 * smo->dt;

        /* Clamp coefficients to +/- 5 degrees in normalized units */
        smo->Ac = saturatef(smo->Ac, smo->max_comp_norm);
        smo->As = saturatef(smo->As, smo->max_comp_norm);
    } else {
        smo->Ac = 0.0f;
        smo->As = 0.0f;
        smo->err_dc = 0.0f;
    }

    /* Freeze PLL integrator accumulation when BEMF magnitude is below ADC noise threshold (standstill) */
    float bemf_sq = smo->Ealpha_flt * smo->Ealpha_flt + smo->Ebeta_flt * smo->Ebeta_flt;
    if (bemf_sq >= 1e-4f) {
        smo->pll_integral += smo->pll_ki * theta_err_clean * smo->dt;
        smo->pll_integral = clampf(smo->pll_integral, smo->pll_int_min, smo->pll_int_max);
    }

    float omega_raw = smo->pll_kp * theta_err_clean + smo->pll_integral;
    float omega_limit = smo->pll_int_max * 1.5f;
    if (omega_limit < 1000.0f) omega_limit = 1000.0f;
    smo->omega_est = saturatef(omega_raw, omega_limit);
    smo->theta_est += (smo->omega_est / PI) * smo->dt;
    smo->theta_est = normalize_angle(smo->theta_est);
}

CCMRAM_FUNC void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha,
                            float Ibeta) {
    float Ialpha_err = smo->Ialpha_est - Ialpha;
    float Ibeta_err = smo->Ibeta_est - Ibeta;
    smo->current_err_sq = Ialpha_err * Ialpha_err + Ibeta_err * Ibeta_err;

    float Zalpha = sigmoid(Ialpha_err, smo->k_sigmoid) * smo->k_slide;
    float Zbeta = sigmoid(Ibeta_err, smo->k_sigmoid) * smo->k_slide;

    smo->Ealpha = Zalpha;
    smo->Ebeta = Zbeta;

    /* Self-Tuning Filter (STF) for BEMF using Backward Euler (unconditionally stable) */
    /* Fast 48kHz LPF at fc = 500Hz (alpha = 2*PI*500Hz/48000Hz = 0.06545f) to remove PLL Kp
     * chattering */
    smo->omega_stf += 0.06545f * (smo->omega_est - smo->omega_stf);
    float omega_stf = smo->omega_stf;
    float f_bw_min = clampf(0.5f * (smo->Rs * smo->Ls_inv * (1.0f / TWO_PI)), 20.0f, 200.0f);
    float f_elec = fabsf(omega_stf) * (1.0f / TWO_PI);
    float wc = TWO_PI * (f_bw_min + 0.8f * f_elec);

    float a = wc * smo->dt;
    float b = omega_stf * smo->dt;
    float D_inv = 1.0f / ((1.0f + a) * (1.0f + a) + b * b);

    float r_alpha = smo->Ealpha_flt + a * smo->Ealpha;
    float r_beta = smo->Ebeta_flt + a * smo->Ebeta;

    smo->Ealpha_flt = ((1.0f + a) * r_alpha - b * r_beta) * D_inv;
    smo->Ebeta_flt = (b * r_alpha + (1.0f + a) * r_beta) * D_inv;

    /* Current observer model: dI/dt = (V - R*I - E) / L */
    /* Using backward Euler: I_new = I_old + dt * dI/dt */
    float dIalpha = (Valpha - smo->Rs * smo->Ialpha_est - smo->Ealpha) * smo->Ls_inv;
    float dIbeta = (Vbeta - smo->Rs * smo->Ibeta_est - smo->Ebeta) * smo->Ls_inv;

    smo->Ialpha_est += dIalpha * smo->dt;
    smo->Ibeta_est += dIbeta * smo->dt;

    SMO_PLL_Track(smo);
}

CCMRAM_FUNC float SMO_GetAngle(SMO_Observer_t* smo) {
    return normalize_angle(smo->theta_est);
}

CCMRAM_FUNC float SMO_GetSpeed(SMO_Observer_t* smo) {
    return smo->omega_est;
}

CCMRAM_FUNC float SMO_GetSpeedRPM(SMO_Observer_t* smo) {
    return (smo->omega_est / smo->poles) * (60.0f / TWO_PI);
}

void SMO_SetMotorParams(SMO_Observer_t* smo, float Rs, float Ls, float flux_linkage, float poles) {
    smo->Rs = Rs;
    smo->Ls = Ls;
    smo->psi = flux_linkage;
    smo->Ls_inv = 1.0f / Ls;
    smo->poles = poles;
}

CCMRAM_FUNC void SMO_FeedBEMF(SMO_Observer_t* smo, float Ealpha, float Ebeta) {
    /* Self-Tuning Filter (STF) for BEMF using Backward Euler (unconditionally stable) */
    /* Fast 48kHz LPF at fc = 500Hz (alpha = 2*PI*500Hz/48000Hz = 0.06545f) to remove PLL Kp
     * chattering */
    smo->omega_stf += 0.06545f * (smo->omega_est - smo->omega_stf);
    float omega_stf = smo->omega_stf;
    float f_bw_min = clampf(0.5f * (smo->Rs * smo->Ls_inv * (1.0f / TWO_PI)), 20.0f, 200.0f);
    float f_elec = fabsf(omega_stf) * (1.0f / TWO_PI);
    float wc = TWO_PI * (f_bw_min + 0.8f * f_elec);

    float a = wc * smo->dt;
    float b = omega_stf * smo->dt;
    float D_inv = 1.0f / ((1.0f + a) * (1.0f + a) + b * b);

    float r_alpha = smo->Ealpha_flt + a * Ealpha;
    float r_beta = smo->Ebeta_flt + a * Ebeta;

    smo->Ealpha_flt = ((1.0f + a) * r_alpha - b * r_beta) * D_inv;
    smo->Ebeta_flt = (b * r_alpha + (1.0f + a) * r_beta) * D_inv;
    SMO_PLL_Track(smo);
}

void SMO_SlowTask(SMO_Observer_t* smo) {
    /* 1. Dynamic PLL Cutoff Frequency based on filtered speed */
    if (smo->omega_est_filt == 0.0f && fabsf(smo->omega_est) > 1.0f) {
        smo->omega_est_filt = fabsf(smo->omega_est);
    } else {
        // Run at 1kHz instead of 48kHz, so adjust LPF coefficient to 0.2f
        smo->omega_est_filt += 0.2f * (fabsf(smo->omega_est) - smo->omega_est_filt);
    }

    float f_elec = smo->omega_est_filt / TWO_PI;
    float pll_cutoff_hz = smo->pll_alpha * f_elec;
    if (pll_cutoff_hz < smo->pll_cutoff_min) pll_cutoff_hz = smo->pll_cutoff_min;
    if (pll_cutoff_hz > 2500.0f) pll_cutoff_hz = 2500.0f;

    smo->pll_kp = 2.0f * TWO_PI * pll_cutoff_hz;
    smo->pll_ki = smo->pll_kp * smo->pll_kp / 4.0f;

    /* 2. SMO Adaptive Sliding Gain Adaptations */
    float omega_obs = 2.0f * PI * g_foc.cfg.bw_cur;
    float R_ratio = omega_obs * smo->Ls - smo->Rs;
    if (R_ratio <= 0.01f) R_ratio = 0.01f;

    float omega_e = fabsf(smo->omega_est);
    float E_est = omega_e * smo->psi;
    smo->k_slide = 1.5f * E_est + 2.0f;

    float k_slide_max = 15.0f * (smo->Ls / smo->dt);
    float safety_net = 2.0f * E_est + 5.0f;
    if (k_slide_max < safety_net) {
        k_slide_max = safety_net;
    }
    if (smo->k_slide > k_slide_max) {
        smo->k_slide = k_slide_max;
    }

    smo->k_sigmoid = smo->k_slide / R_ratio;

    /* 3. Calculate current observer time constant at 1 kHz */
    smo->tau_current = smo->Ls / (smo->Rs + smo->k_slide / smo->k_sigmoid);
}
