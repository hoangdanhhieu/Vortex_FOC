#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#include "foc_config.h"
#include "stm32g4xx_ll_cordic.h"

/*===========================================================================*/
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
void clarke_transform(float Ia, float Ib, float Ic, float* alpha, float* beta);

/**
 * @brief Park transform: Ialpha,Ibeta -> Id,Iq (using electrical angle)
 */
void park_transform(float Ialpha, float Ibeta, float theta, float* Id, float* Iq);

/**
 * @brief Inverse Park transform: Vd,Vq -> Valpha,Vbeta
 */
void inverse_park_transform(float Vd, float Vq, float theta, float* Valpha, float* Vbeta);

/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

/**
 * @brief SVPWM calculation - converts Valpha,Vbeta to duty cycles
 *        Includes internal dead-time compensation based on phase currents.
 * @param Valpha Alpha voltage component
 * @param Vbeta Beta voltage component
 * @param Vbus DC bus voltage
 * @param Ia Phase A current (for dead-time comp sign)
 * @param Ib Phase B current (for dead-time comp sign)
 * @param Ic Phase C current (for dead-time comp sign)
 * @param duty_a Pointer to phase A duty cycle (0.0 to 1.0)
 * @param duty_b Pointer to phase B duty cycle (0.0 to 1.0)
 * @param duty_c Pointer to phase C duty cycle (0.0 to 1.0)
 */
void svpwm_calculate(float* Valpha, float* Vbeta, float Vbus, float Ia, float Ib, float Ic,
                     float* duty_a, float* duty_b, float* duty_c);

/**
 * @brief Set PWM duty cycles to TIM1
 * @param duty_a Phase A duty (0.0 to 1.0)
 * @param duty_b Phase B duty (0.0 to 1.0)
 * @param duty_c Phase C duty (0.0 to 1.0)
 */
void foc_set_pwm_duty(float duty_a, float duty_b, float duty_c);

/*===========================================================================*/
/* ADC Conversion Functions                                                  */
/*===========================================================================*/

/**
 * @brief Reconstruction of phase currents from ADC readings
 *        Selects the best 2 phases based on duty cycle and calculates the 3rd.
 * @param adc_a Raw ADC value for phase A
 * @param adc_b Raw ADC value for phase B
 * @param adc_c Raw ADC value for phase C
 * @param duty_a Duty cycle commanded for phase A
 * @param duty_b Duty cycle commanded for phase B
 * @param duty_c Duty cycle commanded for phase C
 * @param ia Pointer to output current A
 * @param ib Pointer to output current B
 * @param ic Pointer to output current C
 */
void foc_reconstruct_currents(float adc_a, float adc_b, float adc_c, float duty_a, float duty_b,
                              float duty_c, float* ia, float* ib, float* ic);

/**
 * @brief Convert ADC reading to Vbus voltage
 * @param adc_value Raw ADC value (12-bit)
 * @return DC bus voltage in Volts
 */
float foc_adc_to_vbus(uint16_t adc_value);

#endif /* FOC_H */