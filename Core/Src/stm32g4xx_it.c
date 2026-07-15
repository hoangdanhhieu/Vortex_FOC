/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_it.h"

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_config.h"
#include "foc_state_machine.h"

#define HardFault_Handler CubeMX_HardFault_Handler
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t fault_pc = 0;
volatile uint32_t fault_lr = 0;

void HardFault_Handler_C(unsigned long* hardfault_args) {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    FOC_EnableDrivers(0);

    fault_pc = hardfault_args[6]; /* Stacked Program Counter (PC) */
    fault_lr = hardfault_args[5]; /* Stacked Link Register (LR) */

    /* Save to Backup Registers (survives reset) */
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = fault_pc;
    TAMP->BKP1R = fault_lr;

    while (1) {
    }
}

#undef HardFault_Handler
__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile(
        "tst lr, #4 \n"
        "ite eq \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "b HardFault_Handler_C \n");
}
#define HardFault_Handler CubeMX_HardFault_Handler
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN EV */
extern volatile uint16_t raw_adc_a;
extern volatile uint16_t raw_adc_b;
extern volatile uint16_t raw_adc_c;
extern volatile uint16_t raw_adc_vbus;

extern volatile uint32_t adc_isr_us;
extern uint8_t adc_isr_flag;

extern void input_process(void);

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
    /* USER CODE BEGIN BusFault_IRQn 0 */
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    /* USER CODE END BusFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
    /* USER CODE BEGIN UsageFault_IRQn 0 */
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    /* USER CODE END UsageFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 */
void DMA1_Channel2_IRQHandler(void) {
    /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

    /* USER CODE END DMA1_Channel2_IRQn 0 */
    /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

    /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
 * @brief This function handles ADC1 and ADC2 global interrupt.
 */
void ADC1_2_IRQHandler(void) {
    /* USER CODE BEGIN ADC1_2_IRQn 0 */
    if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        uint32_t start_count = TIM2->CNT;
        //  Clear the JEOS flag
        LL_ADC_ClearFlag_JEOS(ADC1);

        // Read ADC1 Injected Data Registers
        raw_adc_a = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        raw_adc_vbus = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);

        // Read ADC2 Injected Data Registers (slave in dual mode)
        raw_adc_b = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        raw_adc_c = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
        // Call FOC control loop
        FOC_HighFrequencyTask(raw_adc_a, raw_adc_b, raw_adc_c, raw_adc_vbus);
        adc_isr_us = TIM2->CNT - start_count;
        if (adc_isr_us > TAMP->BKP2R) {
            TAMP->BKP2R = adc_isr_us;
        }
        TAMP->BKP3R = adc_isr_us;
        adc_isr_flag = 1;
    }
    if (LL_ADC_IsActiveFlag_JEOS(ADC2)) {
        LL_ADC_ClearFlag_JEOS(ADC2);
    }

    /* USER CODE END ADC1_2_IRQn 0 */
    /* USER CODE BEGIN ADC1_2_IRQn 1 */

    /* USER CODE END ADC1_2_IRQn 1 */
}

/**
 * @brief This function handles USB low priority interrupt remap.
 */
void USB_LP_IRQHandler(void) {
    /* USER CODE BEGIN USB_LP_IRQn 0 */

    /* USER CODE END USB_LP_IRQn 0 */
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
    /* USER CODE BEGIN USB_LP_IRQn 1 */

    /* USER CODE END USB_LP_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void) {
    /* USER CODE BEGIN TIM4_IRQn 0 */

    /* USER CODE END TIM4_IRQn 0 */
    /* USER CODE BEGIN TIM4_IRQn 1 */

    /* USER CODE END TIM4_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error
 * interrupts.
 */
void TIM6_DAC_IRQHandler(void) {
    /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        input_process();
    }
    /* USER CODE END TIM6_DAC_IRQn 0 */
    /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

    /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
