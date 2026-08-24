/**
 * @file    peripheral_init.c
 * @brief   Hardware peripheral initialization and configuration for Vortex FOC
 * @note    Contains all clock, GPIO, DMA, ADC, OPAMP, Timer, DAC, UART, and IWDG setups
 */

#include "peripheral_init.h"

#include "foc_config.h"
#include "usb_device.h"
/* Exported variables */
volatile uint16_t adc_regular_buffer[2];

/* Private defines */
#define ADC_THRESHOLD_LOW 200 /* ~5% of 4096, for calibration detection */

/* Private function prototypes */
static uint16_t OPAMP_ReadADC(ADC_TypeDef* ADCx, uint32_t adc_channel);

/* ========================================================================== */
/*                         System Clock & Error Handler                       */
/* ========================================================================== */

/**
 * @brief  System Clock Configuration (170 MHz PLL from HSE 8MHz)
 */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
    }
    LL_PWR_EnableRange1BoostMode();
    LL_RCC_HSE_Enable();
    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {
    }

    LL_RCC_HSI48_Enable();
    /* Wait till HSI48 is ready */
    while (LL_RCC_HSI48_IsReady() != 1) {
    }

    LL_RCC_LSI_Enable();
    /* Wait till LSI is ready */
    while (LL_RCC_LSI_IsReady() != 1) {
    }

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();
    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }

    /* Insure 1us transition state at intermediate medium speed clock*/
    for (__IO uint32_t i = (170 >> 1); i != 0; i--);

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_SetSystemCoreClock(170000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

/* ========================================================================== */
/*                       Low-Level Peripheral Initializers                   */
/* ========================================================================== */

/**
 * @brief GPIO Initialization Function
 */
void MX_GPIO_Init(void) {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    /**/
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

    /**/
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Enable DMA controller clock & IRQ
 */
void MX_DMA_Init(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/**
 * @brief ADC1 Initialization Function
 */
void MX_ADC1_Init(void) {
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
    LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

    /* Peripheral clock enable */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    /**ADC1 GPIO Configuration
    PA0   ------> ADC1_IN1
    PB1   ------> ADC1_IN12
    PB11   ------> ADC1_IN14
    PB12   ------> ADC1_IN11
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */

    /* ADC1 Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

    /* ADC1 interrupt Init */
    NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(ADC1_2_IRQn);

    /** Common config
     */
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_SetGainCompensation(ADC1, 0);
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_INJ_SIMULT;
    ADC_CommonInitStruct.MultiDMATransfer = LL_ADC_MULTI_REG_DMA_EACH_ADC;
    ADC_CommonInitStruct.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_1CYCLE;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
    ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2;
    ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_DISABLE;
    ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
    ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
    LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);
    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC1);
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);
    /* Delay for ADC internal voltage regulator stabilization. */
    /* Compute number of CPU cycles to wait for, from delay in us. */
    /* Note: Variable divided by 2 to compensate partially */
    /* CPU processing cycles (depends on compilation optimization). */
    /* Note: If system core clock frequency is below 200kHz, wait time */
    /* is only a few CPU processing cycles. */
    uint32_t wait_loop_index;
    wait_loop_index =
        ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
        wait_loop_index--;
    }

    /** Configure Analog WatchDog 1
     */
    LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD1, LL_ADC_AWD_CH_VOPAMP1_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, 4095, 0);
    LL_ADC_SetAWDFilteringConfiguration(ADC1, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);
    LL_ADC_EnableIT_AWD1(ADC1);

    /** Configure Injected Channel
     */
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VOPAMP1, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VOPAMP1, LL_ADC_SINGLE_ENDED);

    /** Pre-configure sampling time for CH12 (PB1 = OPAMP3 ext output, used by injected at runtime)
     */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);

    /** Pre-configure sampling time for CH14 (PB11 = Vbus, used by regular single-shot at 1kHz) */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_14, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_14, LL_ADC_SINGLE_ENDED);

    /** Configure Regular Channel Rank 1 (Phase A Voltage: PB12 = ADC1_IN11) */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_11);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);

    /** Configure Regular Channel Rank 2 (Phase C Voltage: PA0 = ADC1_IN1) */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
}

/**
 * @brief ADC2 Initialization Function
 */
void MX_ADC2_Init(void) {
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

    /* Peripheral clock enable */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    /**ADC2 GPIO Configuration
    PC4   ------> ADC2_IN5
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC2 DMA Init */

    /* ADC2 Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_ADC2);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);

    /* ADC2 interrupt Init */
    NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(ADC1_2_IRQn);

    /** Common config
     */
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC2, &ADC_InitStruct);
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
    LL_ADC_SetGainCompensation(ADC2, 0);
    LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
    ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_DISABLE;
    ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
    ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
    LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);
    LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);

    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC2);
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC2);
    /* Delay for ADC internal voltage regulator stabilization. */
    /* Compute number of CPU cycles to wait for, from delay in us. */
    /* Note: Variable divided by 2 to compensate partially */
    /* CPU processing cycles (depends on compilation optimization). */
    /* Note: If system core clock frequency is below 200kHz, wait time */
    /* is only a few CPU processing cycles. */
    uint32_t wait_loop_index;
    wait_loop_index =
        ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
        wait_loop_index--;
    }

    /** Configure Analog WatchDog 1
     */
    LL_ADC_SetAnalogWDMonitChannels(ADC2, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC2, LL_ADC_AWD1, 4095, 0);
    LL_ADC_SetAWDFilteringConfiguration(ADC2, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);
    LL_ADC_EnableIT_AWD1(ADC2);

    /** Configure Injected Channel
     */
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_VOPAMP2, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_VOPAMP2, LL_ADC_SINGLE_ENDED);

    /** Pre-configure sampling time for VOPAMP3 channel (used by runtime channel switching) */
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2, LL_ADC_SINGLE_ENDED);

    /** Configure Regular Channel
     */
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);
}

/**
 * @brief OPAMP1 Initialization Function
 */
void MX_OPAMP1_Init(void) {
    LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**OPAMP1 GPIO Configuration
    PA3   ------> OPAMP1_VINM0
    PA7   ------> OPAMP1_VINP
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED;
    OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_PGA_IO0_BIAS;
    OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO2;
    LL_OPAMP_Init(OPAMP1, &OPAMP_InitStruct);
    LL_OPAMP_SetInputsMuxMode(OPAMP1, LL_OPAMP_INPUT_MUX_DISABLE);
    LL_OPAMP_SetInternalOutput(OPAMP1, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
    LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_8_OR_MINUS_7);
    LL_OPAMP_SetTrimmingMode(OPAMP1, LL_OPAMP_TRIMMING_FACTORY);
}

/**
 * @brief OPAMP2 Initialization Function
 */
void MX_OPAMP2_Init(void) {
    LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    /**OPAMP2 GPIO Configuration
    PA5   ------> OPAMP2_VINM0
    PB0   ------> OPAMP2_VINP
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED;
    OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_PGA_IO0_BIAS;
    OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO2;
    LL_OPAMP_Init(OPAMP2, &OPAMP_InitStruct);
    LL_OPAMP_SetInputsMuxMode(OPAMP2, LL_OPAMP_INPUT_MUX_DISABLE);
    LL_OPAMP_SetInternalOutput(OPAMP2, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
    LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_8_OR_MINUS_7);
    LL_OPAMP_SetTrimmingMode(OPAMP2, LL_OPAMP_TRIMMING_FACTORY);
}

/**
 * @brief OPAMP3 Initialization Function
 */
void MX_OPAMP3_Init(void) {
    LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    /**OPAMP3 GPIO Configuration
    PA1   ------> OPAMP3_VINP
    PB2   ------> OPAMP3_VINM0
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED;
    OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_PGA_IO0_BIAS;
    OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO2;
    LL_OPAMP_Init(OPAMP3, &OPAMP_InitStruct);
    LL_OPAMP_SetInputsMuxMode(OPAMP3, LL_OPAMP_INPUT_MUX_DISABLE);
    LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
    LL_OPAMP_SetPGAGain(OPAMP3, LL_OPAMP_PGA_GAIN_8_OR_MINUS_7);
    LL_OPAMP_SetTrimmingMode(OPAMP3, LL_OPAMP_TRIMMING_FACTORY);
}

/**
 * @brief TIM1 Initialization Function (SVPWM 48kHz & TRGO2)
 */
void MX_TIM1_Init(void) {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP;
    TIM_InitStruct.Autoreload = TIM1_ARR;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM1);
    LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
    TIM_OC_InitStruct.CompareValue = 1600;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4);
    LL_TIM_EnableMasterSlaveMode(TIM1);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = TIM1_DEADTIME_TICKS;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
    TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
    TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
    TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
    TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
    TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**TIM1 GPIO Configuration
    PB13   ------> TIM1_CH1N
    PB14   ------> TIM1_CH2N
    PB15   ------> TIM1_CH3N
    PA8   ------> TIM1_CH1
    PA9   ------> TIM1_CH2
    PA10   ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief CORDIC Initialization Function
 */
void MX_CORDIC_Init(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
}

/**
 * @brief DAC1 Initialization Function
 */
void MX_DAC1_Init(void) {
    LL_DAC_InitTypeDef DAC_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**DAC1 GPIO Configuration
    PA4   ------> DAC1_OUT1
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    LL_DAC_SetHighFrequencyMode(DAC1, LL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ);
    LL_DAC_SetSignedFormat(DAC1, LL_DAC_CHANNEL_1, LL_DAC_SIGNED_FORMAT_DISABLE);
    DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
    DAC_InitStruct.TriggerSource2 = LL_DAC_TRIG_SOFTWARE;
    DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
    DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_DISABLE;
    DAC_InitStruct.OutputConnection = LL_DAC_OUTPUT_CONNECT_GPIO;
    DAC_InitStruct.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
    LL_DAC_Init(DAC1, LL_DAC_CHANNEL_1, &DAC_InitStruct);
    LL_DAC_DisableTrigger(DAC1, LL_DAC_CHANNEL_1);
    LL_DAC_DisableDMADoubleDataMode(DAC1, LL_DAC_CHANNEL_1);
}

/**
 * @brief TIM2 Initialization Function (32-bit Timer)
 */
void MX_TIM2_Init(void) {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    TIM_InitStruct.Prescaler = 169;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 4294967295;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM2);
    LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM2);
}

/**
 * @brief TIM4 Initialization Function (DMA Capture)
 */
void MX_TIM4_Init(void) {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    /**TIM4 GPIO Configuration
    PB7   ------> TIM4_CH2
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM4 DMA Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_TIM4_CH2);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);

    NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(TIM4_IRQn);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 65535;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM4, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM4);
    LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM4);
    LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_BOTHEDGE);
}

/**
 * @brief TIM6 Initialization Function (1kHz Slow Task)
 */
void MX_TIM6_Init(void) {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    TIM_InitStruct.Prescaler = 9;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 16999;
    LL_TIM_Init(TIM6, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM6);
    LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM6);
}

/**
 * @brief USART3 Initialization Function
 */
void MX_USART3_UART_Init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    /**USART3 GPIO Configuration
    PC11   ------> USART3_RX
    PB9   ------> USART3_TX
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART3);
    LL_USART_ConfigAsyncMode(USART3);

    LL_USART_Enable(USART3);

    /* Polling USART3 initialisation */
    while ((!(LL_USART_IsActiveFlag_TEACK(USART3))) || (!(LL_USART_IsActiveFlag_REACK(USART3)))) {
    }
}

/**
 * @brief IWDG Initialization Function
 */
void MX_IWDG_Init(void) {
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32);
    LL_IWDG_SetReloadCounter(IWDG, 50);
    while (LL_IWDG_IsReady(IWDG) != 1) {
    }

    LL_IWDG_ReloadCounter(IWDG);
}

/* ========================================================================== */
/*                       OPAMP & ADC Calibration / Control                    */
/* ========================================================================== */

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
    LL_ADC_ClearFlag_OVR(ADCx);

    return result;
}

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

/**
 * @brief  Reset, reinitialize and start ADC continuous regular + injected conversions
 */
void ADC_Start(void) {
    if (LL_ADC_IsEnabled(ADC1)) {
        LL_ADC_Disable(ADC1);
        while (LL_ADC_IsEnabled(ADC1));
    }
    if (LL_ADC_IsEnabled(ADC2)) {
        LL_ADC_Disable(ADC2);
        while (LL_ADC_IsEnabled(ADC2));
    }

    /* Full RCC reset clears any lingering DMA request state */
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

    /* --- Single-shot 16-sample average measurement of VREFINT at boot --- */
    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);

    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_247CYCLES_5);

    uint32_t vrefint_sum = 0;
    for (int i = 0; i < 16; i++) {
        LL_ADC_REG_StartConversion(ADC1);
        while (!LL_ADC_IsActiveFlag_EOC(ADC1));
        vrefint_sum += LL_ADC_REG_ReadConversionData12(ADC1);
        LL_ADC_ClearFlag_EOC(ADC1);
    }
    uint16_t vrefint_avg = (uint16_t)(vrefint_sum / 16);
    if (vrefint_avg != 0) {
        uint16_t vrefint_cal = *(uint16_t*)0x1FFF75AA;
        ADC_Vref = 3.0f * (float)vrefint_cal / (float)vrefint_avg;
    }

    LL_ADC_Disable(ADC1);
    while (LL_ADC_IsEnabled(ADC1));

    /* Re-init ADC1 with 3-rank phase voltage continuous DMA */
    MX_ADC1_Init();

    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                           LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                           (uint32_t)&adc_regular_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
    LL_ADC_Enable(ADC2);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0);

    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);

    LL_ADC_EnableIT_JEOS(ADC1);
    LL_ADC_INJ_StartConversion(ADC1);

    /* Regular DMA is not enabled at boot (motor starts in IDLE).
     * FOC_SetPhaseVoltageDMA(1) will dynamically configure and start it during CALIBRATION/DETECT.
     */
}

/**
 * @brief  Enable or Disable Phase Voltage Sensing via ADC1 Regular DMA
 */
void FOC_SetPhaseVoltageDMA(uint8_t enable) {
    if (enable) {
        if (!LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1)) {
            /* 1. Stop any ongoing conversion before reconfiguring */
            if (LL_ADC_REG_IsConversionOngoing(ADC1) != 0) {
                LL_ADC_REG_StopConversion(ADC1);
                while (LL_ADC_REG_IsConversionOngoing(ADC1) != 0);
            }

            /* 2. Fully restore ADC1 Regular channel for 2-rank Continuous DMA */
            LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
            LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
            LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_11);
            LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_6CYCLES_5);
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
            LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_6CYCLES_5);

            /* 3. Re-configure and enable DMA */
            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
            LL_DMA_ConfigAddresses(
                DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                (uint32_t)&adc_regular_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
            LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
            LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

            /* 4. Start continuous conversion */
            LL_ADC_REG_StartConversion(ADC1);
        }
    } else {
        if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1)) {
            LL_ADC_REG_StopConversion(ADC1);
            while (LL_ADC_REG_IsConversionOngoing(ADC1) != 0);
            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
        }
    }
}

/**
 * @brief  Read Vbus via software-triggered ADC1 regular single conversion.
 *         Only call when ADC1 regular DMA is DISABLED (i.e. not in DETECT/FLYING_START).
 *         Safe to call from TIM6 ISR (priority 4) — injected conversions (priority 0)
 *         will auto-preempt via STM32G4 hardware injected context queue.
 * @return Raw 12-bit ADC value, or 0 if DMA is active (caller should keep previous value).
 */
uint16_t ADC_ReadVbus_SingleShot(void) {
    /* Don't interfere with regular DMA when it's running */
    if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1)) {
        return 0; /* Caller should keep previous value */
    }

    /* Temporarily configure ADC1 regular for single-shot Vbus */
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_14);

    LL_ADC_REG_StartConversion(ADC1);
    uint32_t timeout = 10000;
    while (!LL_ADC_IsActiveFlag_EOC(ADC1) && --timeout);
    if (timeout == 0) {
        return 0;
    }
    uint16_t result = LL_ADC_REG_ReadConversionData12(ADC1);
    LL_ADC_ClearFlag_EOC(ADC1);
    LL_ADC_ClearFlag_OVR(ADC1);

    return result;
}

/**
 * @brief  Start FOC PWM and Timebase Timers
 */
void TIM_Start(void) {
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);
}

/**
 * @brief  Start Capture Timer
 */
void CAPTURE_Start(void) {
    LL_TIM_EnableCounter(TIM4);
    LL_TIM_EnableDMAReq_CC2(TIM4);
}

/* ========================================================================== */
/*                       Master Peripheral Initialization                     */
/* ========================================================================== */

/**
 * @brief  Initialize all hardware peripherals and calibrate for FOC operation
 */
void Peripheral_Init(void) {
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_OPAMP1_Init();
    MX_TIM1_Init();
    MX_CORDIC_Init();
    MX_OPAMP3_Init();
    MX_OPAMP2_Init();
    MX_TIM2_Init();
    MX_DAC1_Init();
    MX_TIM6_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    MX_USB_Device_Init();

    /* Stabilization delay */
    LL_mDelay(500);

    /* ADC & OPAMP Calibration sequence */
    ADC_Setup();
    OPAMP_Calibration_ADC(OPAMP1, ADC1, LL_ADC_CHANNEL_VOPAMP1);
    OPAMP_Calibration_ADC(OPAMP2, ADC2, LL_ADC_CHANNEL_VOPAMP2);
    OPAMP_Calibration_ADC(OPAMP3, ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2);

    /* Start ADC conversions & Timers */
    ADC_Start();
    TIM_Start();
    // CAPTURE_Start();
}
