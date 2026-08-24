/**
 * @file    stm32g4xx_it.h
 * @brief   Headers of the interrupt handlers for Cortex-M4 and STM32G4 peripherals
 */

#ifndef __STM32G4xx_IT_H
#define __STM32G4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Cortex-M4 Core Interrupt Handlers */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* Peripheral Interrupt Handlers */
void DMA1_Channel2_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void USB_LP_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4xx_IT_H */
