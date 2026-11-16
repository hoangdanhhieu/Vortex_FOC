/**
 * @file input_pot.c
 * @brief Onboard Potentiometer Driver (PC4) Implementation
 */

#include "input_pot.h"

#include "foc_config.h"
#include "stm32g4xx_ll_adc.h"

static uint16_t s_pot_raw = 0;
static float s_pot_raw_filt = 0.0f;
static uint8_t s_pot_armed = 0;
static uint16_t s_zero_counter = 0;
static float s_throttle_norm = 0.0f;

static void Input_Pot_Init(void) {
    s_pot_raw = LL_ADC_REG_ReadConversionData12(ADC2);
    s_pot_raw_filt = (float)s_pot_raw;
    s_pot_armed = 0;
    s_zero_counter = 0;
}

static void Input_Pot_Update(FOC_InputCmd_t* cmd) {
    s_pot_raw = LL_ADC_REG_ReadConversionData12(ADC2);
    s_pot_raw_filt = POT_LPF_ALPHA * s_pot_raw_filt + (1.0f - POT_LPF_ALPHA) * (float)s_pot_raw;

    float norm = s_pot_raw_filt / (float)POT_ADC_MAX;
    norm = clampf(norm, 0.0f, 1.0f);

    const float deadband = 0.10f;

    if (!s_pot_armed) {
        if (norm <= deadband) {
            if (++s_zero_counter >= 100U) {
                s_pot_armed = 1U;
            }
        } else {
            s_zero_counter = 0U;
        }
        s_throttle_norm = 0.0f;
        cmd->arm_req = 0U;
    } else {
        if (norm <= deadband) {
            cmd->arm_req = 0U;
            s_throttle_norm = 0;
        } else {
            s_throttle_norm = (norm - deadband) / (1.0f - deadband);
            s_throttle_norm = clampf(s_throttle_norm, 0.0f, 1.0f);
            cmd->arm_req = 1U;
        }
    }
    cmd->is_active = 1U;
    cmd->is_armed_ready = s_pot_armed;
    cmd->beacon_req = 0U;
}

CCMRAM_FUNC float Pot_ReadThr(void) {
    return s_throttle_norm;
}

const FOC_InputDriver_t g_driver_pot = {.name = "Potentiometer",
                                        .init = Input_Pot_Init,
                                        .update = Input_Pot_Update,
                                        .read_throttle = Pot_ReadThr};
