/**
 * @file foc_hardware.h
 * @brief Hardware Abstraction Layer (HAL) for FOC Fast Path
 *
 * Encapsulates all direct register and STM32 LL driver manipulations for
 * TIM1, ADC1, ADC2, and OPAMP3. All functions are static inline to ensure
 * zero-overhead, zero-wait-state inlining directly into CCMRAM execution.
 */

#ifndef FOC_HARDWARE_H
#define FOC_HARDWARE_H

#include <stdint.h>

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_tim.h"

/*===========================================================================*/
/* PWM Timer (TIM1) Functions                                                */
/*===========================================================================*/

/**
 * @brief Set 3-phase PWM duty cycles directly on TIM1 compare registers
 * @param duty_a Phase A duty (0.0 to 1.0)
 * @param duty_b Phase B duty (0.0 to 1.0)
 * @param duty_c Phase C duty (0.0 to 1.0)
 */
static inline void FOC_HW_SetPWMDuty(float duty_a, float duty_b, float duty_c) {
    float arr = (float)LL_TIM_GetAutoReload(TIM1);
    uint32_t ccr_a = (uint32_t)(duty_a * arr);
    uint32_t ccr_b = (uint32_t)(duty_b * arr);
    uint32_t ccr_c = (uint32_t)(duty_c * arr);

    LL_TIM_OC_SetCompareCH1(TIM1, ccr_a);
    LL_TIM_OC_SetCompareCH2(TIM1, ccr_b);
    LL_TIM_OC_SetCompareCH3(TIM1, ccr_c);
}

/**
 * @brief Immediately disable all TIM1 PWM outputs (zero-overhead inline for ISR use)
 */
static inline void FOC_HW_DisableDrivers(void) {
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 |
                                       LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 |
                                       LL_TIM_CHANNEL_CH3N);
}

/**
 * @brief Enable all TIM1 PWM complementary outputs
 */
static inline void FOC_HW_EnableDrivers(void) {
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 |
                                      LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 |
                                      LL_TIM_CHANNEL_CH3N);
    LL_TIM_EnableAllOutputs(TIM1);
}

/**
 * @brief Enable or disable individual phase gate driver
 * @param phase Phase index (1, 2, or 3)
 * @param enable 1 to enable, 0 to disable
 */
static inline void FOC_HW_EnablePhase(uint8_t phase, uint8_t enable) {
    switch (phase) {
        case 1:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
            }
            break;
        case 2:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
            }
            break;
        case 3:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
            }
            break;
    }
}

/*===========================================================================*/
/* 1Mhz Counter Functions                             */
/*===========================================================================*/
#define FOC_Get_1MhzCounter() (TIM2->CNT)
#define FOC_Reset_1MhzCounter() (TIM2->CNT = 0)

/*===========================================================================*/
/* Injected ADC & OPAMP Multiplexing Functions                                */
/*===========================================================================*/

/**
 * @brief Configure ADC1/ADC2 Injected Sequencer and OPAMP3 for NEXT PWM cycle current sensing.
 *
 * On STM32G4, 3-shunt sensing with 3 OPAMPs uses dynamic phase skipping:
 * - Skip Phase A (0): Measure Phase C via PB1 (ADC1 Ch12) and Phase B via OPAMP2 (ADC2)
 * - Skip Phase B (1): Measure Phase A via OPAMP1 (ADC1) and Phase C via OPAMP3 (ADC2)
 * - Skip Phase C (2): Measure Phase A via OPAMP1 (ADC1) and Phase B via OPAMP2 (ADC2)
 *
 * @param skip_phase Phase to skip (0 = Phase A, 1 = Phase B, 2 = Phase C)
 */
static inline void FOC_HW_SwitchCurrentSensing(uint8_t skip_phase) {
    switch (skip_phase) {
        case 0: /* Skip Physical Phase A (OPAMP1) -> measure C(ADC1 via PB1) + B(ADC2 via VOPAMP2)
                 */
            LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_12);
            LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
            LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
            break;

        case 1: /* Skip Physical Phase B (OPAMP2) -> measure A(ADC1 via VOPAMP1) + C(ADC2 via
                   VOPAMP3) */
            LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
            LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP3_ADC2);
            LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
            break;

        default: /* Skip Physical Phase C (OPAMP3) -> measure A(ADC1 via VOPAMP1) + B(ADC2 via
                    VOPAMP2) */
            LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
            LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
            LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
            break;
    }
}

/**
 * @brief Clear Analog Watchdog flags on ADC1 and ADC2
 */
static inline void FOC_HW_ClearAWDFlags(void) {
    LL_ADC_ClearFlag_AWD1(ADC1);
    LL_ADC_ClearFlag_AWD2(ADC1);
    LL_ADC_ClearFlag_AWD1(ADC2);
    LL_ADC_ClearFlag_AWD2(ADC2);
}

static inline void FOC_TriggerRegularADC(void) {
    if (LL_ADC_REG_IsConversionOngoing(ADC1) == 0) {
        LL_ADC_REG_StartConversion(ADC1);
    }
}
#endif /* FOC_HARDWARE_H */
