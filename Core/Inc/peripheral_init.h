/**
 * @file    peripheral_init.h
 * @brief   User peripheral initialization functions
 * @note    Contains OPAMP calibration and ADC setup functions
 */

#ifndef __PERIPHERAL_INIT_H
#define __PERIPHERAL_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
/* Exported variables --------------------------------------------------------*/
extern volatile uint16_t adc_regular_buffer[2];
/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  OPAMP Self-Calibration using ADC (for internal output mode)
 * @note   When OPAMP output is internally connected to ADC (OPAINTOEN=1),
 *         CALOUT bit cannot be used. Must use ADC to detect output change.
 * @param  OPAMPx: OPAMP instance (OPAMP1, OPAMP2, or OPAMP3)
 * @param  ADCx: ADC connected to OPAMP (ADC1 for OPAMP1, ADC2 for OPAMP2/3)
 * @param  adc_channel: ADC channel for OPAMP internal output
 */
void OPAMP_Calibration_ADC(OPAMP_TypeDef* OPAMPx, ADC_TypeDef* ADCx, uint32_t adc_channel);

/**
 * @brief  Setup ADC for FOC operation after OPAMP calibration
 * @note   Resets ADC, reinitializes, and starts injected conversions
 */
void ADC_Setup(void);

/**
 * @brief  Initialize all peripherals for FOC operation
 * @note   Call this after MX_xxx_Init() functions in main()
 *         Performs: ADC calibration, OPAMP calibration, ADC setup for FOC
 */
void Peripheral_Init(void);

void TIM_Start(void);
#ifdef __cplusplus
}
#endif

#endif /* __PERIPHERAL_INIT_H */
