/**
 * @file    main.c
 * @brief   Main application entry point for Vortex FOC
 */

#include "main.h"

#include "comm_protocol.h"
#include "flash_config.h"
#include "foc_hardware.h"
#include "foc_state_machine.h"
#include "peripheral_init.h"

#define WDG_CHECK_TIME 20000
/* Global telemetry & debug variables */
volatile uint16_t raw_adc_a = 0;
volatile uint16_t raw_adc_b = 0;
volatile uint16_t raw_adc_c = 0;

volatile uint32_t adc_isr_us = 0;
volatile uint32_t reset_cause = 0;
volatile uint32_t saved_pc = 0;
volatile uint32_t saved_lr = 0;
volatile uint32_t saved_max_isr_cycles = 0;
volatile uint32_t saved_last_isr_cycles = 0;
volatile uint32_t saved_r0 = 0;
volatile uint32_t saved_r1 = 0;
volatile uint32_t saved_r2 = 0;
volatile uint32_t saved_r3 = 0;
volatile uint32_t saved_r12 = 0;
volatile uint32_t saved_psr = 0;

volatile uint16_t g_adc_ticks = 0;
volatile uint8_t g_slow_ticks = 0;
static uint32_t last_wdg_check = 0;

volatile float ADC_Vref = 3.3f;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* Reset of all peripherals, Initializes Flash interface and Systick */

    HAL_Init();

    /* Configure 170 MHz system clock */
    SystemClock_Config();

    /* Read and clear reset cause */
    reset_cause = RCC->CSR;
    LL_RCC_ClearResetFlags();

    /* Read backup registers to recover crash info (survives reset) */
    HAL_PWR_EnableBkUpAccess();
    saved_pc = TAMP->BKP0R;
    saved_lr = TAMP->BKP1R;
    saved_max_isr_cycles = TAMP->BKP2R;
    saved_last_isr_cycles = TAMP->BKP3R;
    saved_r0 = TAMP->BKP8R;
    saved_r1 = TAMP->BKP9R;
    saved_r2 = TAMP->BKP10R;
    saved_r3 = TAMP->BKP11R;
    saved_r12 = TAMP->BKP12R;
    saved_psr = TAMP->BKP13R;
    __NOP();

    /* Clear backup registers to avoid stale crash info */
    TAMP->BKP0R = 0;
    TAMP->BKP1R = 0;
    TAMP->BKP2R = 0;
    TAMP->BKP3R = 0;
    TAMP->BKP8R = 0;
    TAMP->BKP9R = 0;
    TAMP->BKP10R = 0;
    TAMP->BKP11R = 0;
    TAMP->BKP12R = 0;
    TAMP->BKP13R = 0;
    TAMP->BKP14R = 0;

    /* Initialize Flash config (load from Flash or defaults) */
    FlashConfig_Init();

    /* Initialize all hardware peripherals & calibrate ADC/OPAMP */
    Peripheral_Init();

    /* Initialize FOC controller & apply parameters */
    FOC_Init();
    FlashConfig_Apply();

    /* Initialize Communication Protocol */
    Comm_Init();
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);  // Enable DAC for plotting

    /* Start Independent Watchdog */
    MX_IWDG_Init();

    g_adc_ticks = 0;
    g_slow_ticks = 0;
    last_wdg_check = FOC_Get_1MhzCounter();
    while (1) {
        Comm_Update();
        uint32_t now = FOC_Get_1MhzCounter();
        if (now - last_wdg_check >= WDG_CHECK_TIME) {
            const float time_duration = (float)(now - last_wdg_check) * 0.000001f;
            const uint32_t adc_target = (uint32_t)(time_duration / FOC_GetDt());
            const uint32_t slow_target = (uint32_t)(time_duration / 0.001f);
            const uint32_t adc_count_min = adc_target * 85 / 100;
            const uint32_t adc_count_max = adc_target * 115 / 100;
            const uint32_t slow_count_min = slow_target * 85 / 100;
            const uint32_t slow_count_max = slow_target * 115 / 100;
            uint16_t adc_count = g_adc_ticks;
            uint8_t slow_count = g_slow_ticks;
            g_adc_ticks = 0;
            g_slow_ticks = 0;
            last_wdg_check = now;
            if ((adc_count >= adc_count_min && adc_count <= adc_count_max) &&
                (slow_count >= slow_count_min && slow_count <= slow_count_max)) {
                LL_IWDG_ReloadCounter(IWDG);
            }
        }
    }
}
