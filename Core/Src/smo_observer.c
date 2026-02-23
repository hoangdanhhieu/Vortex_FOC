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
    float abs_x = (x >= 0.0f) ? x : -x;
    return x / (abs_x + k);
}

/**
 * @brief Normalize angle to [-1, 1) range (corresponds to [-pi, pi))
 */
CCMRAM_FUNC static inline float normalize_angle(float angle) {
    while (angle >= 1.0f) {
        angle -= 2.0f;
    }
    while (angle < -1.0f) {
        angle += 2.0f;
    }
    return angle;
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

    /* Calculate LPF coefficient from cutoff frequency */
    /* flt_coeff = dt / (dt + tau) */
    /* tau = 1/(2*PI*f_cutoff_Hz) — cutoff frequency in Hz needs 2π conversion */
    smo->tau = 1.0f / (TWO_PI * SMO_BEMF_CUTOFF);
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

CCMRAM_FUNC void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha,
                            float Ibeta) {
    float Ialpha_err = smo->Ialpha_est - Ialpha;
    float Ibeta_err = smo->Ibeta_est - Ibeta;

    float Zalpha = sigmoid(Ialpha_err, smo->k_sigmoid) * smo->k_slide;
    float Zbeta = sigmoid(Ibeta_err, smo->k_sigmoid) * smo->k_slide;

    smo->Ealpha = Zalpha;
    smo->Ebeta = Zbeta;

    smo->Ealpha_flt += smo->lpf_coeff * (smo->Ealpha - smo->Ealpha_flt);
    smo->Ebeta_flt += smo->lpf_coeff * (smo->Ebeta - smo->Ebeta_flt);

    /* Current observer model: dI/dt = (V - R*I - E) / L */
    /* Using backward Euler: I_new = I_old + dt * dI/dt */
    float dIalpha = (Valpha - smo->Rs * smo->Ialpha_est - smo->Ealpha) * smo->Ls_inv;
    float dIbeta = (Vbeta - smo->Rs * smo->Ibeta_est - smo->Ebeta) * smo->Ls_inv;

    smo->Ialpha_est += dIalpha * smo->dt;
    smo->Ibeta_est += dIbeta * smo->dt;

    /* Angle estimation using atan2 on filtered back-EMF */
    /* Note: BEMF leads current by 90°, so we use -Ealpha, -Ebeta */
    /* The BEMF is: E = -d(flux)/dt, and for PMSM: E_alpha = -omega * flux *
     * sin(theta) */
    /*                                             E_beta = omega * flux *
     * cos(theta)  */
    /* So theta = atan2(-Ealpha, Ebeta) */
    float theta_bemf = cordic_atan2(-smo->Ealpha_flt, smo->Ebeta_flt);

    /* Phase compensation for LPF lag */
    /* lag = atan(omega * tau) */
    /* Use CORDIC for atan2(omega*tau, 1.0) to get result in [-1, 1) range */
    /* Note: cordic_atan2 return value is normalized [-1, 1) */
    /* atan(x) returns rad, we need normalized. cordic_atan2(y, x) handles it. */
    float phase_lag = cordic_atan2(smo->omega_est * smo->tau, 1.0f);

    float theta_comp = theta_bemf + phase_lag;
    theta_comp = normalize_angle(theta_comp);

    /* PLL for smooth angle tracking */
    /* Use phase-compensated angle for PLL error */
    float theta_err = theta_comp - smo->theta_est;

    /* Normalize error to [-1, 1) */
    theta_err = normalize_angle(theta_err);

    /* PLL PI controller */
    smo->pll_integral += smo->pll_ki * theta_err * smo->dt;

    /* Clamp PLL integral to prevent windup */
    if (smo->pll_integral > smo->pll_int_max) {
        smo->pll_integral = smo->pll_int_max;
    } else if (smo->pll_integral < smo->pll_int_min) {
        smo->pll_integral = smo->pll_int_min;
    }

    smo->omega_est = smo->pll_kp * theta_err + smo->pll_integral;

    /* Integrate speed to get angle */
    /* omega_est is in rad/s, need to convert to normalized angle rate */
    /* d(theta_norm)/dt = omega / PI */
    smo->theta_est += (smo->omega_est / PI) * smo->dt;
    smo->theta_est = normalize_angle(smo->theta_est);
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

void SMO_SetFilterParams(SMO_Observer_t* smo, float bemf_cutoff_hz, float pll_cutoff_hz) {
    smo->tau = 1.0f / (TWO_PI * bemf_cutoff_hz);
    smo->lpf_coeff = CONTROL_PERIOD / (CONTROL_PERIOD + smo->tau);

    smo->pll_kp = 2.0f * TWO_PI * pll_cutoff_hz;
    smo->pll_ki = smo->pll_kp * smo->pll_kp / 4.0f;
}
