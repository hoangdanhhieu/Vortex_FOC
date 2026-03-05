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
    angle += 1.0f;
    angle -= 2.0f * (float)((int)(angle * 0.5f));
    if (angle >= 2.0f) angle -= 2.0f;
    if (angle < 0.0f) angle += 2.0f;
    return angle - 1.0f;
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

    /* Cache motor parameters */
    smo->Rs = MOTOR_RS;
    smo->Ls = MOTOR_LS;
    smo->psi = MOTOR_FLUX_LINKAGE;
    smo->Ls_inv = 1.0f / MOTOR_LS;

    /* Sample time */
    smo->dt = CONTROL_PERIOD;
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
    smo->pll_integral = 0.0f;
}

CCMRAM_FUNC static inline void SMO_PLL_Track(SMO_Observer_t* smo) {
    float theta_bemf = cordic_atan2(-smo->Ealpha_flt, smo->Ebeta_flt);
    float phase_lag = smo->omega_est * smo->tau / PI;
    float theta_comp = normalize_angle(theta_bemf + phase_lag);
    float theta_err = normalize_angle(theta_comp - smo->theta_est);

    smo->pll_integral += smo->pll_ki * theta_err * smo->dt;
    if (smo->pll_integral > smo->pll_int_max) {
        smo->pll_integral = smo->pll_int_max;
    } else if (smo->pll_integral < smo->pll_int_min) {
        smo->pll_integral = smo->pll_int_min;
    }

    smo->omega_est = smo->pll_kp * theta_err + smo->pll_integral;
    smo->theta_est += (smo->omega_est / PI) * smo->dt;
    smo->theta_est = normalize_angle(smo->theta_est);
}

CCMRAM_FUNC void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha,
                            float Ibeta) {
    float Ialpha_err = smo->Ialpha_est - Ialpha;
    float Ibeta_err = smo->Ibeta_est - Ibeta;

    float Zalpha = sigmoid(Ialpha_err, smo->k_sigmoid) * smo->k_slide;
    float Zbeta = sigmoid(Ibeta_err, smo->k_sigmoid) * smo->k_slide;

    smo->Ealpha = Zalpha;
    smo->Ebeta = Zbeta;

    smo->tau = 1.0f / (fabsf(smo->omega_est) * 2.0f + 60.0f);
    smo->lpf_coeff = CONTROL_PERIOD / (CONTROL_PERIOD + smo->tau);
    smo->Ealpha_flt += smo->lpf_coeff * (smo->Ealpha - smo->Ealpha_flt);
    smo->Ebeta_flt += smo->lpf_coeff * (smo->Ebeta - smo->Ebeta_flt);

    /* Current observer model: dI/dt = (V - R*I - E) / L */
    /* Using backward Euler: I_new = I_old + dt * dI/dt */
    float dIalpha = (Valpha - smo->Rs * smo->Ialpha_est - smo->Ealpha) * smo->Ls_inv;
    float dIbeta = (Vbeta - smo->Rs * smo->Ibeta_est - smo->Ebeta) * smo->Ls_inv;

    smo->Ialpha_est += dIalpha * smo->dt;
    smo->Ibeta_est += dIbeta * smo->dt;

    SMO_PLL_Track(smo);
}

float SMO_GetAngle(SMO_Observer_t* smo) {
    return normalize_angle(smo->theta_est);
}

float SMO_GetSpeed(SMO_Observer_t* smo) {
    return smo->omega_est;
}

float SMO_GetSpeedRPM(SMO_Observer_t* smo) {
    return (smo->omega_est / smo->poles) * (60.0f / TWO_PI);
}

void SMO_SetGains(SMO_Observer_t* smo, float k_slide, float k_sigmoid) {
    smo->k_slide = k_slide;
    smo->k_sigmoid = k_sigmoid;
}

void SMO_SetMotorParams(SMO_Observer_t* smo, float Rs, float Ls, float flux_linkage, float poles) {
    smo->Rs = Rs;
    smo->Ls = Ls;
    smo->psi = flux_linkage;
    smo->Ls_inv = 1.0f / Ls;
    smo->poles = poles;
}

void SMO_SetFilterParams(SMO_Observer_t* smo, float pll_cutoff_hz) {
    smo->pll_kp = 2.0f * TWO_PI * pll_cutoff_hz;
    smo->pll_ki = smo->pll_kp * smo->pll_kp / 4.0f;
}

CCMRAM_FUNC void SMO_FeedBEMF(SMO_Observer_t* smo, float Ealpha, float Ebeta) {
    smo->tau = 1.0f / (fabsf(smo->omega_est) * 2.0f + 60.0f);
    smo->lpf_coeff = CONTROL_PERIOD / (CONTROL_PERIOD + smo->tau);
    smo->Ealpha_flt += smo->lpf_coeff * (Ealpha - smo->Ealpha_flt);
    smo->Ebeta_flt += smo->lpf_coeff * (Ebeta - smo->Ebeta_flt);
    SMO_PLL_Track(smo);
}
