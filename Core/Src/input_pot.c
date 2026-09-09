/**
 * @file input_pot.c
 * @brief Onboard Potentiometer Driver (PC4) Implementation
 */

#include "input_pot.h"

#include "foc_config.h"
#include "foc_state_machine.h"
#include "peripheral_init.h"

static uint16_t s_pot_raw = 0;
static float s_pot_raw_filt = 0.0f;

static void Input_Pot_Init(void) {
    s_pot_raw = ADC_ReadPot_SingleShot();
    s_pot_raw_filt = (float)s_pot_raw;
}

static void Input_Pot_Read(FOC_InputCmd_t* cmd) {
    s_pot_raw = ADC_ReadPot_SingleShot();
    s_pot_raw_filt = POT_LPF_ALPHA * s_pot_raw_filt + (1.0f - POT_LPF_ALPHA) * (float)s_pot_raw;

    float norm = s_pot_raw_filt / (float)POT_ADC_MAX;
    norm = clampf(norm, 0.0f, 1.0f);

    float deadband = clampf(g_foc.cfg.input_deadband, 0.01f, 0.40f);
    if (norm <= deadband) {
        cmd->throttle = 0.0f;
        cmd->arm_req = 0;
    } else {
        float active_range = 1.0f - deadband;
        cmd->throttle = (active_range > 0.001f) ? ((norm - deadband) / active_range) : 0.0f;
        cmd->throttle = clampf(cmd->throttle, 0.0f, 1.0f);
        cmd->arm_req = (norm > deadband * 1.5f) ? 1 : 0;
    }

    cmd->is_active = 1;
    cmd->dir_req = 0;
}

const FOC_InputDriver_t g_driver_pot = {
    .name = "Potentiometer",
    .init = Input_Pot_Init,
    .read = Input_Pot_Read
};
