/**
 * @file    peripheral_init.h
 * @brief   Hardware peripheral initialization and configuration functions for Vortex FOC
 */

#ifndef __PERIPHERAL_INIT_H
#define __PERIPHERAL_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Exported variables */
extern volatile uint16_t adc_regular_buffer[2];

/* High-level System Initialization */
void SystemClock_Config(void);
void Peripheral_Init(void);

/* OPAMP & ADC Calibration / Setup */
void OPAMP_Calibration_ADC(OPAMP_TypeDef* OPAMPx, ADC_TypeDef* ADCx, uint32_t adc_channel);
void ADC_Setup(void);
void ADC_Start(void);
void FOC_SetPhaseVoltageDMA(uint8_t enable);
uint16_t ADC_ReadVbus_SingleShot(void);
uint16_t ADC_ReadPot_SingleShot(void);

/* Timer & Capture Controls */
void TIM_Start(void);
void CAPTURE_Start(void);

/* Individual Peripheral Initializers */
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_OPAMP1_Init(void);
void MX_OPAMP2_Init(void);
void MX_OPAMP3_Init(void);
void MX_TIM1_Init(void);
void MX_CORDIC_Init(void);
void MX_TIM2_Init(void);
void MX_TIM4_Init(void);
void MX_TIM6_Init(void);
void MX_DAC1_Init(void);
void MX_USART3_UART_Init(void);
void MX_IWDG_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPHERAL_INIT_H */
