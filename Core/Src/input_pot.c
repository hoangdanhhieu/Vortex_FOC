/**
 * @file input_pot.c
 * @brief Onboard Potentiometer Driver (PC4) Implementation
 */

#include "input_pot.h"

#include "foc_config.h"
#include "foc_state_machine.h"
#include "stm32g4xx_ll_adc.h"

static uint16_t s_pot_raw = 0;
static float s_pot_raw_filt = 0.0f;
static uint8_t s_pot_armed = 0;
static uint16_t s_zero_counter = 0;

static void Input_Pot_Init(void) {
    s_pot_raw = LL_ADC_REG_ReadConversionData12(ADC2);
    s_pot_raw_filt = (float)s_pot_raw;
    s_pot_armed = 0;
    s_zero_counter = 0;
}

static void Input_Pot_Read(FOC_InputCmd_t* cmd) {
    s_pot_raw = LL_ADC_REG_ReadConversionData12(ADC2);
    s_pot_raw_filt = POT_LPF_ALPHA * s_pot_raw_filt + (1.0f - POT_LPF_ALPHA) * (float)s_pot_raw;

    float norm = s_pot_raw_filt / (float)POT_ADC_MAX;
    norm = clampf(norm, 0.0f, 1.0f);

    float deadband = clampf(FOC_GetConfigInputDeadband(), 0.01f, 0.40f);

    /* Zero-throttle safety interlock: hold throttle <= deadband for 100 ms (100 ticks at 1 kHz) */
    if (!s_pot_armed) {
        if (norm <= deadband) {
            if (++s_zero_counter >= 100U) {
                s_pot_armed = 1U;
            }
        } else {
            s_zero_counter = 0U;
        }
        cmd->throttle = 0.0f;
        cmd->arm_req = 0U;
    } else {
        if (norm <= deadband) {
            cmd->throttle = 0.0f;
            cmd->arm_req = 0U;
        } else {
            float active_range = 1.0f - deadband;
            cmd->throttle = (active_range > 0.001f) ? ((norm - deadband) / active_range) : 0.0f;
            cmd->throttle = clampf(cmd->throttle, 0.0f, 1.0f);
            cmd->arm_req = 1U;
        }
    }

    cmd->is_active = 1U;
    cmd->dir_req = 1;
}

const FOC_InputDriver_t g_driver_pot = {
    .name = "Potentiometer",
    .is_high_speed = 0,
    .init = Input_Pot_Init,
    .read = Input_Pot_Read
};
