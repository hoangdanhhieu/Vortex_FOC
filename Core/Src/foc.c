#include "foc.h"

#include <main.h>

#include "cordic_math.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "stm32g4xx_ll_tim.h"

/*===========================================================================*/
/* Clarke/Park Transforms                                                    */
/*===========================================================================*/

CCMRAM_FUNC void clarke_transform(float Ia, float Ib, float Ic, float* alpha, float* beta) {
    /* Amplitude-invariant Clarke transform */
    *alpha = Ia;
    *beta = (Ia + 2.0f * Ib) * SQRT3_INV;
}

CCMRAM_FUNC void park_transform(float Ialpha, float Ibeta, float theta, float* Id, float* Iq) {
    float sin_theta, cos_theta;
    cordic_sincos(theta, &cos_theta, &sin_theta);

    *Id = Ialpha * cos_theta + Ibeta * sin_theta;
    *Iq = -Ialpha * sin_theta + Ibeta * cos_theta;
}

CCMRAM_FUNC void inverse_park_transform(float Vd, float Vq, float theta, float* Valpha,
                                        float* Vbeta) {
    float sin_theta, cos_theta;
    cordic_sincos(theta, &cos_theta, &sin_theta);

    *Valpha = Vd * cos_theta - Vq * sin_theta;
    *Vbeta = Vd * sin_theta + Vq * cos_theta;
}

/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

/*
 * Dead-time compensation constant (Duty cycle equivalent)
 * D_dead = T_dead / T_pwm = T_dead * F_pwm
 * Factor 2.0 covers total loss (rise+fall or effective pulse width reduction)
 */
#define DEAD_TIME_DUTY (DEAD_TIME_NS * 1e-9f * (float)PWM_FREQUENCY * 2.0f)
CCMRAM_FUNC void svpwm_calculate(float* Valpha, float* Vbeta, float Vbus, float Ia, float Ib,
                                 float Ic, float* duty_a, float* duty_b, float* duty_c) {
    /* Safety: Prevent division by zero */
    if (Vbus < 1.0f) {
        Vbus = 1.0f; /* Minimum valid voltage */
    }

    /* Normalize voltages to Vbus */
    float Vbus_inv = 1.0f / Vbus;
    float va_norm = *Valpha * Vbus_inv;
    float vb_norm = *Vbeta * Vbus_inv;

    /* Clamp voltage vector magnitude to SVPWM linear range */
    /* Max radius of inscribed circle in voltage hexagon = 1/sqrt(3) */
    float v_mag = cordic_modulus(va_norm, vb_norm);
    const float V_MAX_NORM = SQRT3_INV; /* ~0.5774 */
    if (v_mag > V_MAX_NORM) {
        float scale = V_MAX_NORM / v_mag;
        va_norm *= scale;
        vb_norm *= scale;
    }

    *Valpha = va_norm * Vbus;
    *Vbeta = vb_norm * Vbus;

    /* Calculate phase voltages using inverse Clarke */
    float Va = va_norm;
    float Vb = -0.5f * va_norm + (SQRT3 * 0.5f) * vb_norm;
    float Vc = -0.5f * va_norm - (SQRT3 * 0.5f) * vb_norm;

    /* Find min and max for midpoint clamping (SVPWM) */
    float Vmin = Va;
    float Vmax = Va;

    if (Vb < Vmin) Vmin = Vb;
    if (Vc < Vmin) Vmin = Vc;
    if (Vb > Vmax) Vmax = Vb;
    if (Vc > Vmax) Vmax = Vc;

    /* Midpoint offset for centered PWM (symmetric SVPWM) */
    float Voffset = (Vmax + Vmin) * 0.5f;

    /* Calculate initial duty cycles (0.5 centered) */
    /* Range 0.0 to 1.0 */
    float da = (Va - Voffset) + 0.5f;
    float db = (Vb - Voffset) + 0.5f;
    float dc = (Vc - Voffset) + 0.5f;

    /* Apply Dead-time Compensation directly to Duty Cycles */
    const float I_threshold = 0.5f;
    float dt_comp = DEAD_TIME_DUTY;

    if (Ia > I_threshold)
        da += dt_comp;
    else if (Ia < -I_threshold)
        da -= dt_comp;

    if (Ib > I_threshold)
        db += dt_comp;
    else if (Ib < -I_threshold)
        db -= dt_comp;

    if (Ic > I_threshold)
        dc += dt_comp;
    else if (Ic < -I_threshold)
        dc -= dt_comp;

    /* Clamp duty cycles to valid range [0, 1] */
    if (da < 0.0f) da = 0.0f;
    if (da > g_foc.max_duty) da = g_foc.max_duty;
    if (db < 0.0f) db = 0.0f;
    if (db > g_foc.max_duty) db = g_foc.max_duty;
    if (dc < 0.0f) dc = 0.0f;
    if (dc > g_foc.max_duty) dc = g_foc.max_duty;

    *duty_a = da;
    *duty_b = db;
    *duty_c = dc;
}

CCMRAM_FUNC void foc_set_pwm_duty(float duty_a, float duty_b, float duty_c) {
    /* Convert duty cycles to compare values */
    uint32_t ccr_a = (uint32_t)(duty_a * (float)TIM1_ARR);
    uint32_t ccr_b = (uint32_t)(duty_b * (float)TIM1_ARR);
    uint32_t ccr_c = (uint32_t)(duty_c * (float)TIM1_ARR);

    /* Update TIM1 compare registers */
    LL_TIM_OC_SetCompareCH1(TIM1, ccr_a);
    LL_TIM_OC_SetCompareCH2(TIM1, ccr_b);
    LL_TIM_OC_SetCompareCH3(TIM1, ccr_c);
}

/*===========================================================================*/
/* ADC Conversion Functions                                                  */
/*===========================================================================*/

CCMRAM_FUNC void foc_reconstruct_currents(float i_a, float i_b, float i_c, float duty_a,
                                          float duty_b, float duty_c, float* ia_out, float* ib_out,
                                          float* ic_out) {
    float i_meas_a = i_a;
    float i_meas_b = i_b;
    float i_meas_c = i_c;

    float max_duty = duty_a;
    int max_index = 0; /* 0=A, 1=B, 2=C */

    if (duty_b > max_duty) {
        max_duty = duty_b;
        max_index = 1;
    }
    if (duty_c > max_duty) {
        max_duty = duty_c;
        max_index = 2;
    }

    switch (max_index) {
        case 0: /* A has highest duty -> Drop A */
            *ib_out = i_meas_b;
            *ic_out = i_meas_c;
            *ia_out = -(*ib_out + *ic_out);
            break;
        case 1: /* B has highest duty -> Drop B */
            *ia_out = i_meas_a;
            *ic_out = i_meas_c;
            *ib_out = -(*ia_out + *ic_out);
            break;
        case 2: /* C has highest duty -> Drop C */
            *ia_out = i_meas_a;
            *ib_out = i_meas_b;
            *ic_out = -(*ia_out + *ib_out);
            break;
    }
}

CCMRAM_FUNC float foc_adc_to_vbus(uint16_t adc_value) {
    /* Vbus = ADC * conversion_factor (with divider ratio) */
    return (float)adc_value * ADC_TO_VBUS;
}