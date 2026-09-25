#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#include "foc_config.h"
#include "foc_hardware.h"

/*===========================================================================*/
/* Clarke/Park Transforms                                                    */
/*===========================================================================*/

CCMRAM_FUNC static inline void clarke_transform(float Ia, float Ib, float Ic, float* alpha,
                                                float* beta) {
    *alpha = Ia;
    *beta = (Ib - Ic) * SQRT3_INV;
}

/**
 * @brief Inverse Clarke transform: Ialpha,Ibeta -> Ia,Ib,Ic
 */
CCMRAM_FUNC static inline void inverse_clarke_transform(float alpha, float beta, float* Ia,
                                                        float* Ib, float* Ic) {
    *Ia = alpha;
    *Ib = -0.5f * alpha + 0.8660254f * beta;
    *Ic = -(*Ia + *Ib);
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

/**
 * @brief Normalize angle to [-1, 1) range (corresponds to [-pi, pi))
 */
CCMRAM_FUNC static inline float normalize_angle_norm(float angle) {
    float y = (angle + 1.0f) * 0.5f + 64.0f;
    float flr = (float)((int32_t)y) - 64.0f;
    return angle - 2.0f * flr;
}

/**
 * @brief Self-Tuning Filter (STF) step in alpha-beta frame (Backward Euler)
 * @param in_alpha  Alpha input signal
 * @param in_beta   Beta input signal
 * @param flt_alpha Pointer to filtered alpha state variable (updated in-place)
 * @param flt_beta  Pointer to filtered beta state variable (updated in-place)
 * @param wc        Cutoff angular frequency [rad/s]
 * @param omega     Center tracking angular frequency [rad/s]
 * @param dt        Sample period [s]
 */
CCMRAM_FUNC static inline void stf_filter_step(float in_alpha, float in_beta, float* flt_alpha,
                                               float* flt_beta, float wc, float omega, float dt) {
    float a = wc * dt;
    float b = omega * dt;
    float D_inv = 1.0f / ((1.0f + a) * (1.0f + a) + b * b);

    float r_alpha = *flt_alpha + a * in_alpha;
    float r_beta = *flt_beta + a * in_beta;

    *flt_alpha = ((1.0f + a) * r_alpha - b * r_beta) * D_inv;
    *flt_beta = (b * r_alpha + (1.0f + a) * r_beta) * D_inv;
}

/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

/**
 * @brief SVPWM calculation - converts Valpha,Vbeta to duty cycles
 *        Includes internal dead-time compensation based on phase currents.
 */
void svpwm_calculate(float theta);

/**
 * @brief Set PWM duty cycles to TIM1
 * @param duty_a Phase A duty (0.0 to 1.0)
 * @param duty_b Phase B duty (0.0 to 1.0)
 * @param duty_c Phase C duty (0.0 to 1.0)
 */
CCMRAM_FUNC static inline void foc_set_pwm_duty(float duty_a, float duty_b, float duty_c) {
    FOC_HW_SetPWMDuty(duty_a, duty_b, duty_c);
}

/*===========================================================================*/
/* ADC Conversion Functions                                                  */
/*===========================================================================*/

/**
 * @brief Convert ADC reading to Vbus voltage
 * @param adc_value Raw ADC value (12-bit)
 * @return DC bus voltage in Volts
 */
CCMRAM_FUNC static inline float foc_adc_to_vbus(uint16_t adc_value) {
    return (float)adc_value * ADC_TO_VBUS;
}

/**
 * @brief Convert ADC reading to Phase voltage using 3-resistor bias divider formula (differential
 * with offset)
 * @param adc_value Raw ADC value (12-bit)
 * @param adc_offset Offset ADC value calibrated at zero phase voltage
 * @return Phase voltage in Volts (can be positive or negative)
 */
CCMRAM_FUNC static inline float foc_adc_to_vphase(uint16_t adc_value, uint16_t adc_offset) {
    float adc_diff = (float)adc_value - (float)adc_offset;
    return adc_diff * ADC_Vref * (PHASE_VOLTAGE_GAIN / (float)ADC_RESOLUTION);
}

/**
 * @brief Convert ADC reading to absolute Phase voltage (without offset subtraction)
 * @param adc_value Raw ADC value (12-bit)
 * @return Absolute Phase voltage in Volts (referenced to MCU GND)
 */
CCMRAM_FUNC static inline float foc_adc_to_vphase_absolute(uint16_t adc_value) {
    float v_adc = (float)adc_value * (ADC_Vref / (float)ADC_RESOLUTION);
    return v_adc * PHASE_VOLTAGE_GAIN - ADC_Vref * PHASE_VOLTAGE_OFFSET_FACTOR;
}

/**
 * @brief Apply centralized deadtime compensation to duty cycles.
 */
void foc_apply_deadtime_compensation(float* out_a, float* out_b, float* out_c);

#endif /* FOC_H */