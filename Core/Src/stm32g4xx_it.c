/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines for Cortex-M4 and STM32G4 peripherals
 */

#include "stm32g4xx_it.h"

#include "foc_config.h"
#include "foc_state_machine.h"
#include "main.h"

/* External handles */
extern PCD_HandleTypeDef hpcd_USB_FS;

/* External telemetry / control variables */
extern volatile uint16_t raw_adc_a;
extern volatile uint16_t raw_adc_b;
extern volatile uint16_t raw_adc_c;
extern volatile uint32_t adc_isr_us;
extern uint8_t adc_isr_flag;

/* Fault debug variables */
volatile uint32_t fault_pc = 0;
volatile uint32_t fault_lr = 0;
volatile uint16_t adc1_data;
volatile uint16_t adc2_data;
/* ========================================================================== */
/*           Cortex-M4 Processor Exception Handlers                           */
/* ========================================================================== */

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  C-level HardFault handler to log registers into backup SRAM
 * @param  hardfault_args: pointer to stacked exception frame
 */
void HardFault_Handler_C(unsigned long* hardfault_args) {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    FOC_EnableDrivers(0);

    fault_pc = hardfault_args[6]; /* Stacked Program Counter (PC) */
    fault_lr = hardfault_args[5]; /* Stacked Link Register (LR) */

    /* Save to Backup Registers (survives reset) */
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = fault_pc;
    TAMP->BKP1R = fault_lr;
    TAMP->BKP8R = hardfault_args[0];  /* R0 */
    TAMP->BKP9R = hardfault_args[1];  /* R1 */
    TAMP->BKP10R = hardfault_args[2]; /* R2 */
    TAMP->BKP11R = hardfault_args[3]; /* R3 */
    TAMP->BKP12R = hardfault_args[4]; /* R12 */
    TAMP->BKP13R = hardfault_args[7]; /* PSR */

    while (1) {
    }
}

/**
 * @brief  HardFault Naked Assembly wrapper
 */
__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile(
        "tst lr, #4 \n"
        "ite eq \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "b HardFault_Handler_C \n");
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    while (1) {
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    while (1) {
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    while (1) {
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
    HAL_IncTick();
}

/* ========================================================================== */
/*                 STM32G4xx Peripheral Interrupt Handlers                    */
/* ========================================================================== */

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 */
void DMA1_Channel2_IRQHandler(void) {
}

/**
 * @brief This function handles ADC1 and ADC2 global interrupt (FOC Fast Loop 48kHz).
 */

void ADC1_2_IRQHandler(void) {
    // Check AWD1 on ADC1 (Phase A overcurrent)
    if (LL_ADC_IsActiveFlag_AWD1(ADC1)) {
        LL_ADC_ClearFlag_AWD1(ADC1);
        FOC_EnableDrivers(0);
        g_foc.status.fault = FOC_FAULT_OVERCURRENT;
        g_foc.status.state = FOC_STATE_FAULT;
    }

    // Check AWD1 on ADC2 (Phase B and C overcurrent)
    if (LL_ADC_IsActiveFlag_AWD1(ADC2)) {
        LL_ADC_ClearFlag_AWD1(ADC2);
        FOC_EnableDrivers(0);
        g_foc.status.fault = FOC_FAULT_OVERCURRENT;
        g_foc.status.state = FOC_STATE_FAULT;
    }

    if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        uint32_t start_count = TIM2->CNT;
        LL_ADC_ClearFlag_JEOS(ADC1);

        /* Read 1 rank from each ADC (dual simultaneous) */
        adc1_data = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        adc2_data = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);

        FOC_HighFrequencyTask(adc1_data, adc2_data);
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
}

/**
 * @brief This function handles USB low priority interrupt remap.
 */
void USB_LP_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void) {
}

/**
 * @brief This function handles TIM6 global interrupt and DAC underrun (1kHz FOC Task).
 */
void TIM6_DAC_IRQHandler(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) {
        LL_TIM_ClearFlag_UPDATE(TIM6);

        FOC_SlowTask();
        SMO_SlowTask(&g_foc.ctrl.smo);
    }
}
