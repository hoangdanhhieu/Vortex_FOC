/**
 * @file    main.h
 * @brief   Header for main application and peripheral configuration
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_iwdg.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_utils.h"

/* Exported variables */
extern volatile float ADC_Vref;

/* Exported functions prototypes */
void Error_Handler(void);

#define LED_Pin LL_GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
