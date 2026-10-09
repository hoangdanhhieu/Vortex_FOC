#include "foc_input.h"

#include <stddef.h>

#include "foc_state_machine.h"
#include "input_dshot.h"
#include "input_pot.h"

static FOC_InputSource_t s_input_source = FOC_INPUT_SOURCE_NONE;
static const FOC_InputDriver_t* s_custom_driver = NULL;
static const FOC_InputDriver_t* s_active_driver = NULL;

static float s_throttle_active = 0.0f;
static float s_dynamic_min_vq = 0.0f;
static uint8_t s_armed = 0;
static uint16_t s_fault_clear_count = 0;

static void FOC_Input_SwitchDriver(const FOC_InputDriver_t* new_driver,
                                   FOC_InputSource_t new_source) {
    if (s_active_driver != new_driver) {
        s_active_driver = new_driver;
        s_input_source = new_source;
        s_armed = 0;
        s_fault_clear_count = 0;
        s_throttle_active = 0.0f;
        if (s_active_driver && s_active_driver->init) {
            s_active_driver->init();
        }
    } else {
        s_input_source = new_source;
    }
}

void FOC_Input_Init(void) {
    s_armed = 0;
    s_fault_clear_count = 0;
    s_throttle_active = 0.0f;
    s_dynamic_min_vq = 0.0f;
    s_active_driver = NULL;
    s_input_source = FOC_INPUT_SOURCE_NONE;

    uint8_t src_cfg = FOC_GetConfigInputSource();
    if (src_cfg == FOC_INPUT_SOURCE_DSHOT) {
        FOC_Input_SwitchDriver(&g_driver_dshot, FOC_INPUT_SOURCE_DSHOT);
    } else if (src_cfg == FOC_INPUT_SOURCE_CUSTOM && s_custom_driver != NULL) {
        FOC_Input_SwitchDriver(s_custom_driver, FOC_INPUT_SOURCE_CUSTOM);
    } else if (src_cfg == FOC_INPUT_SOURCE_POT) {
        FOC_Input_SwitchDriver(&g_driver_pot, FOC_INPUT_SOURCE_POT);
    } else if (src_cfg == FOC_INPUT_SOURCE_UART_USB) {
        FOC_Input_SwitchDriver(NULL, FOC_INPUT_SOURCE_UART_USB);
    } else {
        FOC_Input_SwitchDriver(NULL, FOC_INPUT_SOURCE_NONE);
    }
}

void FOC_Input_SetMinVq(float vq_min) {
    s_dynamic_min_vq = vq_min;
}

void FOC_Input_SelectSource(FOC_InputSource_t source) {
    s_input_source = source;
}

void FOC_Input_RegisterCustomDriver(const FOC_InputDriver_t* driver) {
    s_custom_driver = driver;
    if (s_custom_driver && s_custom_driver->init) {
        s_custom_driver->init();
    }
}

FOC_InputSource_t FOC_Input_GetSource(void) {
    return s_input_source;
}

float FOC_Input_GetThrottle(void) {
    return s_throttle_active;
}

uint8_t FOC_Input_IsArmed(void) {
    return s_armed;
}

/**
 * @brief Map a normalized throttle [0..1] onto the active control-mode target.
 *
 * Single source of truth for throttle-to-target mapping, shared by the 1 kHz
 * slow-path dispatch and the 48 kHz high-speed driver fast path.
 */
CCMRAM_FUNC static void FOC_Input_MapThrottle(float thr) {
    switch (FOC_GetControlMode()) {
        case FOC_MODE_SPEED: {
            float min_spd = FOC_GetInputMinSpd();
            float max_spd = FOC_GetMaxSpeed();
            FOC_SetSpeedRef(min_spd + thr * (max_spd - min_spd));
            break;
        }
        case FOC_MODE_TORQUE: {
            float min_cur = FOC_GetInputMinCur();
            float max_cur = FOC_GetMaxCurrent();
            FOC_SetTorqueCurrent(min_cur + thr * (max_cur - min_cur));
            break;
        }
        case FOC_MODE_VOLTAGE: {
            float min_vq = FOC_GetInputMinVq();
            if (s_dynamic_min_vq > min_vq) {
                min_vq = s_dynamic_min_vq;
            }
            FOC_SetVoltageRef((min_vq + thr * (1.0f - min_vq)) * 100.0f);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief Resolve the USB override and select the active physical driver.
 * @param cmd On return, holds the latest driver command when a physical driver is active.
 * @return 1 when input is disabled or handed over to the host (caller must stop),
 *         0 when a physical driver is active.
 */
static uint8_t FOC_Input_SelectActiveDriver(FOC_InputCmd_t* cmd) {
    uint8_t src_cfg = FOC_GetConfigInputSource();

    /* Host GUI Control via USB or Input Disabled */
    if (src_cfg == FOC_INPUT_SOURCE_UART_USB) {
        FOC_Input_SwitchDriver(NULL, FOC_INPUT_SOURCE_UART_USB);
        return 1;
    }
    if (src_cfg == FOC_INPUT_SOURCE_NONE) {
        FOC_Input_SwitchDriver(NULL, FOC_INPUT_SOURCE_NONE);
        return 1;
    }

    /* Select Active Physical Driver based on Config */
    if (src_cfg == FOC_INPUT_SOURCE_CUSTOM && s_custom_driver != NULL) {
        FOC_Input_SwitchDriver(s_custom_driver, FOC_INPUT_SOURCE_CUSTOM);
    } else if (src_cfg == FOC_INPUT_SOURCE_DSHOT) {
        FOC_Input_SwitchDriver(&g_driver_dshot, FOC_INPUT_SOURCE_DSHOT);
    } else {
        FOC_Input_SwitchDriver(&g_driver_pot, FOC_INPUT_SOURCE_POT);
    }

    /* Read Hardware */
    FOC_InputCmd_t zero = {0};
    *cmd = zero;
    if (s_active_driver && s_active_driver->read) {
        s_active_driver->read(cmd);
    }
    s_throttle_active = cmd->throttle;
    return 0;
}

/**
 * @brief Update the arming state (1 kHz).
 *
 * Each active driver owns its safety handshake / zero interlock and reports
 * arming readiness via cmd->arm_req. If the system is in FAULT, arming is forced to 0.
 */
static void FOC_Input_UpdateArming(const FOC_InputCmd_t* cmd, FOC_State_t state) {
    if (state == FOC_STATE_FAULT) {
        s_armed = 0;
        return;
    }
    s_armed = cmd->is_active ? cmd->arm_req : 0U;
}

/**
 * @brief Stop the motor and reset the input-layer state (1 kHz).
 */
static void FOC_Input_DoStop(void) {
    FOC_SetVoltageRef(0.0f);
    FOC_SetSpeedRef(0.0f);
    FOC_SetTorqueCurrent(0.0f);
    FOC_Stop();
    s_armed = 0;
    s_dynamic_min_vq = 0.0f;
}

/**
 * @brief Zero-throttle fault recovery (1 kHz).
 *
 * When FOC is in FAULT, holding the throttle at zero continuously for 200 ms
 * acknowledges and clears the fault condition, transitioning FOC back to IDLE.
 */
static void FOC_Input_CheckFaultRecovery(const FOC_InputCmd_t* cmd, FOC_State_t state) {
    if (state != FOC_STATE_FAULT) {
        s_fault_clear_count = 0;
        return;
    }

    if (!cmd->arm_req && cmd->is_active) {
        if (++s_fault_clear_count >= 200U) { /* 200 ms continuous zero throttle */
            FOC_ClearFault();
            s_fault_clear_count = 0;
        }
    } else {
        s_fault_clear_count = 0;
    }
}

/**
 * @brief Apply the driver's direction request (1 kHz).
 *
 * dir_req: +1 = FWD, -1 = REV, 0 = keep current. When a reversal is requested,
 * stops the motor, applies the new direction, and triggers immediate FOC restart
 * so the core's BEMF detector & dynamic brake (FOC_STATE_BRAKE) safely reverse rotation.
 */
static void FOC_Input_CheckDirection(const FOC_InputCmd_t* cmd, FOC_State_t state) {
    if (cmd->dir_req == 0 || cmd->dir_req == FOC_GetDirection()) {
        return;
    }
    if (state != FOC_STATE_IDLE && state != FOC_STATE_STOP && state != FOC_STATE_FAULT &&
        state != FOC_STATE_BEEP) {
        FOC_Input_DoStop();
    }
    FOC_SetDirection(cmd->dir_req);
    if (cmd->arm_req && cmd->is_active) {
        uint8_t mode_val = FOC_GetConfigInputMode();
        if (mode_val > 2) mode_val = 2;
        FOC_SetControlMode((FOC_ControlMode_t)mode_val);
        s_armed = 1;
        FOC_Start();
    }
}

/**
 * @brief Evaluate the start trigger and the stop/failsafe triggers (1 kHz).
 */
static void FOC_Input_CheckStartStop(const FOC_InputCmd_t* cmd, FOC_State_t state) {
    /* Start Trigger: apply configured input_mode and start motor.
     * A start during the power-on beep preempts it (FOC_Start aborts it). */
    if ((state == FOC_STATE_IDLE || state == FOC_STATE_BEEP) && s_armed && cmd->arm_req &&
        cmd->is_active) {
        uint8_t mode_val = FOC_GetConfigInputMode();
        if (mode_val > 2) mode_val = 2;
        FOC_SetControlMode((FOC_ControlMode_t)mode_val);
        FOC_Start();
    }

    /* Stop Trigger & Failsafe. The beep is a parked state: a missing arm
     * signal (e.g. pot at zero during the power-on beep) must NOT abort it. */
    if (state != FOC_STATE_IDLE && state != FOC_STATE_FAULT && state != FOC_STATE_STOP &&
        state != FOC_STATE_BEEP) {
        if (!cmd->arm_req || !cmd->is_active) {
            FOC_Input_DoStop();
        }
    }
}

void FOC_Input_Update(void) {
    FOC_InputCmd_t cmd;
    if (FOC_Input_SelectActiveDriver(&cmd)) {
        return;
    }

    FOC_State_t state = FOC_GetState();
    FOC_Input_CheckFaultRecovery(&cmd, state);
    state = FOC_GetState(); /* Re-sample state if fault was cleared to IDLE */
    FOC_Input_UpdateArming(&cmd, state);
    FOC_Input_CheckDirection(&cmd, state);
    state = FOC_GetState(); /* A direction-induced stop may have moved us to IDLE */
    FOC_Input_CheckStartStop(&cmd, state);

    if (state == FOC_STATE_RUN && cmd.is_active &&
        !(s_active_driver && s_active_driver->is_high_speed)) {
        FOC_Input_MapThrottle(cmd.throttle);
    }
}

CCMRAM_FUNC void FOC_Input_Update_HF(void) {
    if (!s_armed || s_active_driver == NULL || !s_active_driver->is_high_speed ||
        s_active_driver->read_fast == NULL) {
        return;
    }
    s_throttle_active = s_active_driver->read_fast();
    FOC_Input_MapThrottle(s_throttle_active);
}
