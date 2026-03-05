/**
 * @file    peripheral_init.c
 * @brief   User peripheral initialization functions
 * @note    Contains OPAMP calibration and ADC setup functions for FOC
 */

#include "peripheral_init.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_utils.h"

/* External functions from main.c (MX_xxx_Init) */
extern void MX_ADC1_Init(void);
extern void MX_ADC2_Init(void);

/* Exported variables --------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define ADC_THRESHOLD_LOW 200 /* ~5% of 4096, for calibration detection */

/* Private function prototypes -----------------------------------------------*/
static uint16_t OPAMP_ReadADC(ADC_TypeDef* ADCx, uint32_t adc_channel);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Read ADC value from OPAMP internal output for calibration
 * @note   Uses REGULAR conversion (simpler than injected for single reads)
 * @param  ADCx: ADC instance connected to OPAMP
 * @param  adc_channel: ADC channel connected to OPAMP internal output
 * @return ADC value (0-4095)
 */
static uint16_t OPAMP_ReadADC(ADC_TypeDef* ADCx, uint32_t adc_channel) {
    LL_ADC_REG_SetContinuousMode(ADCx, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADCx, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_REG_SetSequencerRanks(ADCx, LL_ADC_REG_RANK_1, adc_channel);
    LL_ADC_REG_SetSequencerLength(ADCx, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_SetChannelSamplingTime(ADCx, adc_channel, LL_ADC_SAMPLINGTIME_47CYCLES_5);

    LL_ADC_REG_StartConversion(ADCx);
    while (!LL_ADC_IsActiveFlag_EOC(ADCx));
    uint16_t result = LL_ADC_REG_ReadConversionData12(ADCx);
    LL_ADC_ClearFlag_EOC(ADCx);

    return result;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  OPAMP Self-Calibration using ADC (for internal output mode)
 */
void OPAMP_Calibration_ADC(OPAMP_TypeDef* OPAMPx, ADC_TypeDef* ADCx, uint32_t adc_channel) {
    volatile uint32_t trimming_val_nmos = 16;
    volatile uint32_t trimming_val_pmos = 16;
    uint16_t adc_val;

    if (!LL_OPAMP_IsEnabled(OPAMPx)) {
        LL_OPAMP_Enable(OPAMPx);
        LL_mDelay(1);
    }

    LL_OPAMP_SetTrimmingMode(OPAMPx, LL_OPAMP_TRIMMING_USER);

    LL_OPAMP_SetMode(OPAMPx, LL_OPAMP_MODE_CALIBRATION);
    LL_mDelay(5);

    LL_OPAMP_SetCalibrationSelection(OPAMPx, LL_OPAMP_TRIMMING_NMOS);
    LL_mDelay(2);

    for (uint32_t i = 0; i <= 31; i++) {
        LL_OPAMP_SetTrimmingValue(OPAMPx, LL_OPAMP_TRIMMING_NMOS, i);
        LL_mDelay(1);

        adc_val = OPAMP_ReadADC(ADCx, adc_channel);

        if (adc_val < ADC_THRESHOLD_LOW) {
            trimming_val_nmos = i;
            break;
        }
    }

    LL_OPAMP_SetCalibrationSelection(OPAMPx, LL_OPAMP_TRIMMING_PMOS);
    LL_mDelay(2);

    for (uint32_t i = 0; i <= 31; i++) {
        LL_OPAMP_SetTrimmingValue(OPAMPx, LL_OPAMP_TRIMMING_PMOS, i);
        LL_mDelay(1);

        adc_val = OPAMP_ReadADC(ADCx, adc_channel);

        if (adc_val < ADC_THRESHOLD_LOW) {
            trimming_val_pmos = i;
            break;
        }
    }

    LL_OPAMP_SetMode(OPAMPx, LL_OPAMP_MODE_FUNCTIONAL);

    if ((trimming_val_nmos >= 2) && (trimming_val_nmos <= 29) && (trimming_val_pmos >= 2) &&
        (trimming_val_pmos <= 29)) {
        LL_OPAMP_SetTrimmingMode(OPAMPx, LL_OPAMP_TRIMMING_USER);
        LL_OPAMP_SetTrimmingValue(OPAMPx, LL_OPAMP_TRIMMING_NMOS, trimming_val_nmos);
        LL_OPAMP_SetTrimmingValue(OPAMPx, LL_OPAMP_TRIMMING_PMOS, trimming_val_pmos);
    } else {
        LL_OPAMP_SetTrimmingMode(OPAMPx, LL_OPAMP_TRIMMING_FACTORY);
    }

    LL_mDelay(1);
}

/**
 * @brief  Setup ADC for FOC operation after OPAMP calibration
 */
void ADC_Setup(void) {
    if (LL_ADC_IsEnabled(ADC1)) {
        LL_ADC_Disable(ADC1);
        while (LL_ADC_IsEnabled(ADC1));
    }
    if (LL_ADC_IsEnabled(ADC2)) {
        LL_ADC_Disable(ADC2);
        while (LL_ADC_IsEnabled(ADC2));
    }

    LL_mDelay(1);
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0);
    LL_mDelay(1);
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
    LL_mDelay(1);

    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
    LL_ADC_Enable(ADC2);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0);
}

void ADC_Start() {
    if (LL_ADC_IsEnabled(ADC1)) {
        LL_ADC_Disable(ADC1);
        while (LL_ADC_IsEnabled(ADC1));
    }
    if (LL_ADC_IsEnabled(ADC2)) {
        LL_ADC_Disable(ADC2);
        while (LL_ADC_IsEnabled(ADC2));
    }

    LL_AHB2_GRP1_ForceReset(RCC_AHB2RSTR_ADC12RST);
    __NOP();
    LL_AHB2_GRP1_ReleaseReset(RCC_AHB2RSTR_ADC12RST);
    __NOP();
    MX_ADC1_Init();
    MX_ADC2_Init();

    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
    LL_mDelay(1);
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0);
    LL_mDelay(1);

    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                           LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                           (uint32_t)&adc_regular_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 4);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
    LL_ADC_Enable(ADC2);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0);

    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);

    LL_ADC_EnableIT_JEOS(ADC1);
    LL_ADC_INJ_StartConversion(ADC1);
    LL_ADC_REG_StartConversion(ADC1);
}

void TIM_Start(void) {
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);
}

void CAPTURE_Start() {
    LL_TIM_EnableCounter(TIM4);
    LL_TIM_EnableDMAReq_CC2(TIM4);
}

/**
 * @brief  Initialize all peripherals for FOC operation
 */
void Peripheral_Init(void) {
    ADC_Setup();
    OPAMP_Calibration_ADC(OPAMP1, ADC1, LL_ADC_CHANNEL_VOPAMP1);
    OPAMP_Calibration_ADC(OPAMP2, ADC2, LL_ADC_CHANNEL_VOPAMP2);
    OPAMP_Calibration_ADC(OPAMP3, ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2);
    ADC_Start();
    TIM_Start();
    CAPTURE_Start();
}
