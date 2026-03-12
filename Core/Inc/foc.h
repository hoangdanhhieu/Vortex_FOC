#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#include "foc_config.h"
#include "foc_state_machine.h"
#include "stm32g4xx_ll_tim.h"
/* Q31 and Angle Conversion Functions                                        */
/*===========================================================================*/

int32_t radians_to_q31(float radians);
int32_t degrees_to_q31(float degrees);
int32_t float_to_q31(float value);
float q31_to_float(int32_t q31_value);

/*===========================================================================*/
/* CORDIC Accelerated Functions                                              */
/*===========================================================================*/

void cordic_sin_cos(int32_t angle, int32_t* sin, int32_t* cos);
void cordic_sin_cos_f32(float theta, float* sin_out, float* cos_out);

/*===========================================================================*/
/* Clarke/Park Transforms                                                    */
/*===========================================================================*/

/**
 * @brief Clarke transform: Ia,Ib,Ic -> Ialpha,Ibeta
 */
CCMRAM_FUNC static inline void clarke_transform(float Ia, float Ib, float* alpha, float* beta) {
    *alpha = Ia;
    *beta = (Ia + 2.0f * Ib) * SQRT3_INV;
}

/**
 * @brief Park transform: Ialpha,Ibeta -> Id,Iq
 */
CCMRAM_FUNC static inline void park_transform(float Ialpha, float Ibeta, float cos_th, float sin_th,
                                              float* Id, float* Iq) {
    *Id = Ialpha * cos_th + Ibeta * sin_th;
    *Iq = -Ialpha * sin_th + Ibeta * cos_th;
}

/**
 * @brief Inverse Park transform: Vd,Vq -> Valpha,Vbeta
 */
CCMRAM_FUNC static inline void inverse_park_transform(float Vd, float Vq, float cos_th,
                                                      float sin_th, float* Valpha, float* Vbeta) {
    *Valpha = Vd * cos_th - Vq * sin_th;
    *Vbeta = Vd * sin_th + Vq * cos_th;
}

/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

/**
 * @brief SVPWM calculation - converts Valpha,Vbeta to duty cycles
 *        Includes internal dead-time compensation based on phase currents.
 */
void svpwm_calculate(void);

/**
 * @brief Set PWM duty cycles to TIM1
 * @param duty_a Phase A duty (0.0 to 1.0)
 * @param duty_b Phase B duty (0.0 to 1.0)
 * @param duty_c Phase C duty (0.0 to 1.0)
 */
CCMRAM_FUNC static inline void foc_set_pwm_duty(float duty_a, float duty_b, float duty_c) {
    uint32_t ccr_a = (uint32_t)(duty_a * (float)TIM1_ARR);
    uint32_t ccr_b = (uint32_t)(duty_b * (float)TIM1_ARR);
    uint32_t ccr_c = (uint32_t)(duty_c * (float)TIM1_ARR);

    LL_TIM_OC_SetCompareCH1(TIM1, ccr_a);
    LL_TIM_OC_SetCompareCH2(TIM1, ccr_b);
    LL_TIM_OC_SetCompareCH3(TIM1, ccr_c);
}

/*===========================================================================*/
/* ADC Conversion Functions                                                  */
/*===========================================================================*/

/**
 * @brief Reconstruction of phase currents from ADC readings
 *        Selects the best 2 phases based on duty cycle and calculates the 3rd.
 */
CCMRAM_FUNC static inline void foc_reconstruct_currents(void) {
    float max_duty = g_foc.data.duty_a;
    int max_index = 0; /* 0=A, 1=B, 2=C */

    if (g_foc.data.duty_b > max_duty) {
        max_duty = g_foc.data.duty_b;
        max_index = 1;
    }
    if (g_foc.data.duty_c > max_duty) {
        max_duty = g_foc.data.duty_c;
        max_index = 2;
    }

    if (max_index == 0) { /* A has highest duty -> Drop A */
        g_foc.data.Ia = -(g_foc.data.Ib + g_foc.data.Ic);
    } else if (max_index == 1) { /* B has highest duty -> Drop B */
        g_foc.data.Ib = -(g_foc.data.Ia + g_foc.data.Ic);
    } else { /* C has highest duty -> Drop C */
        g_foc.data.Ic = -(g_foc.data.Ia + g_foc.data.Ib);
    }
}

/**
 * @brief Convert ADC reading to Vbus voltage
 * @param adc_value Raw ADC value (12-bit)
 * @return DC bus voltage in Volts
 */
CCMRAM_FUNC static inline float foc_adc_to_vbus(uint16_t adc_value) {
    return (float)adc_value * ADC_TO_VBUS;
}

#endif /* FOC_H */