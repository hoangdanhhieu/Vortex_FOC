#include "foc_input.h"

#include <stddef.h>

#include "foc_state_machine.h"
#include "input_pot.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

static FOC_InputSource_t s_input_source = FOC_INPUT_SOURCE_NONE;
static const FOC_InputDriver_t* s_custom_driver = NULL;
static const FOC_InputDriver_t* s_active_driver = &g_driver_pot;

static float s_throttle_active = 0.0f;
static float s_dynamic_min_vq = 0.0f;
static uint8_t s_armed = 0;
static uint16_t s_zero_count = 0;

void FOC_Input_Init(void) {
    s_armed = 0;
    s_zero_count = 0;
    s_throttle_active = 0.0f;
    s_dynamic_min_vq = 0.0f;
    s_input_source = FOC_INPUT_SOURCE_POT;
    s_active_driver = &g_driver_pot;
    if (g_driver_pot.init) {
        g_driver_pot.init();
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
static void FOC_Input_MapThrottle(float thr) {
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
    /* USB Override: Top Priority for Tuning / Debugging */
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        s_input_source = FOC_INPUT_SOURCE_UART_USB;
        s_active_driver = NULL;
        s_armed = 0;
        s_zero_count = 0;
        s_throttle_active = 0.0f;
        return 1;
    }

    /* Select Active Physical Driver based on Config */
    uint8_t src_cfg = FOC_GetConfigInputSource();
    if (src_cfg == FOC_INPUT_SOURCE_NONE) {
        s_input_source = FOC_INPUT_SOURCE_NONE;
        s_active_driver = NULL;
        s_armed = 0;
        s_zero_count = 0;
        s_throttle_active = 0.0f;
        return 1;
    }

    if (src_cfg == FOC_INPUT_SOURCE_CUSTOM && s_custom_driver != NULL) {
        s_active_driver = s_custom_driver;
        s_input_source = FOC_INPUT_SOURCE_CUSTOM;
    } else {
        s_active_driver = &g_driver_pot;
        s_input_source = FOC_INPUT_SOURCE_POT;
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
 * Drivers with manages_arming=1 are the source of truth for arming
 * (e.g. DShot protocol arming). All other drivers use the pot-style
 * zero-throttle lock: hold throttle at 0 for 100 ms to arm.
 */
static void FOC_Input_UpdateArming(const FOC_InputCmd_t* cmd, FOC_State_t state) {
    if (s_active_driver && s_active_driver->manages_arming) {
        s_armed = ((state == FOC_STATE_IDLE || state == FOC_STATE_FAULT) ? 0u : cmd->arm_req);
        return;
    }

    if (state == FOC_STATE_IDLE || state == FOC_STATE_FAULT) {
        if (state == FOC_STATE_FAULT) {
            s_armed = 0;
            s_zero_count = 0;
        }
        if (!s_armed) {
            if (cmd->throttle <= 0.001f) {
                if (++s_zero_count >= 100) { /* 100ms at zero throttle */
                    s_armed = 1;
                }
            } else {
                s_zero_count = 0;
            }
        }
    }
}

/**
 * @brief Evaluate the start trigger and the stop/failsafe triggers (1 kHz).
 */
static void FOC_Input_CheckStartStop(const FOC_InputCmd_t* cmd, FOC_State_t state) {
    /* Start Trigger: apply configured input_mode and start motor */
    if (state == FOC_STATE_IDLE && s_armed && cmd->arm_req && cmd->is_active) {
        uint8_t mode_val = FOC_GetConfigInputMode();
        if (mode_val > 2) mode_val = 2;
        FOC_SetControlMode((FOC_ControlMode_t)mode_val);
        FOC_Start();
    }

    /* Stop Trigger & Failsafe */
    if (state != FOC_STATE_IDLE && state != FOC_STATE_FAULT && state != FOC_STATE_STOP) {
        if (!cmd->arm_req || !cmd->is_active) {
            FOC_SetVoltageRef(0.0f);
            FOC_SetSpeedRef(0.0f);
            FOC_SetTorqueCurrent(0.0f);
            FOC_Stop();
            s_armed = 0;
            s_zero_count = 0;
            s_dynamic_min_vq = 0.0f;
        }
    }
}

void FOC_Input_Update(void) {
    FOC_InputCmd_t cmd;
    if (FOC_Input_SelectActiveDriver(&cmd)) {
        return;
    }

    FOC_State_t state = FOC_GetState();
    FOC_Input_UpdateArming(&cmd, state);
    FOC_Input_CheckStartStop(&cmd, state);

    /* Target Dispatcher when running (slow drivers only; high-speed drivers
     * are dispatched at 48 kHz via FOC_Input_ApplyFastTarget_HF). */
    if (state == FOC_STATE_RUN && cmd.is_active &&
        !(s_active_driver && s_active_driver->is_high_speed)) {
        FOC_Input_MapThrottle(cmd.throttle);
    }
}

CCMRAM_FUNC void FOC_Input_ApplyFastTarget_HF(void) {
    if (!s_armed || s_active_driver == NULL || !s_active_driver->is_high_speed ||
        s_active_driver->read_fast == NULL) {
        return;
    }
    s_throttle_active = s_active_driver->read_fast();
    FOC_Input_MapThrottle(s_throttle_active);
}
