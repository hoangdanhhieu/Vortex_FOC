#include "foc_input.h"

#include <stddef.h>

#include "foc_config.h"
#include "foc_state_machine.h"
#include "input_pot.h"
#include "peripheral_init.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/*===========================================================================*/
/* State Variables                                                           */
/*===========================================================================*/

static FOC_InputSource_t s_input_source = FOC_INPUT_SOURCE_NONE;
static const FOC_InputDriver_t* s_custom_driver = NULL;

static float s_throttle_active = 0.0f;
static float s_dynamic_min_vq = 0.0f;
static uint8_t s_armed = 0;
static uint16_t s_zero_count = 0;

/*===========================================================================*/
/* Public APIs                                                               */
/*===========================================================================*/

void FOC_Input_Init(void) {
    s_armed = 0;
    s_zero_count = 0;
    s_throttle_active = 0.0f;
    s_dynamic_min_vq = 0.0f;
    s_input_source = FOC_INPUT_SOURCE_POT;
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

void FOC_Input_Update(void) {
    /* 1. USB Override: Top Priority for Tuning / Debugging */
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        s_input_source = FOC_INPUT_SOURCE_UART_USB;
        s_armed = 0;
        s_zero_count = 0;
        s_throttle_active = 0.0f;
        return;
    }

    /* 2. Select Active Physical Driver based on Config */
    uint8_t src_cfg = (uint8_t)(g_foc.cfg.input_source + 0.5f);
    if (src_cfg == 0) {
        /* Input disabled */
        s_input_source = FOC_INPUT_SOURCE_NONE;
        s_armed = 0;
        s_zero_count = 0;
        s_throttle_active = 0.0f;
        return;
    }

    const FOC_InputDriver_t* driver = NULL;
    if (src_cfg == 2 && s_custom_driver != NULL) {
        driver = s_custom_driver;
        s_input_source = FOC_INPUT_SOURCE_CUSTOM;
    } else {
        driver = &g_driver_pot;
        s_input_source = FOC_INPUT_SOURCE_POT;
    }

    /* 3. Read Hardware */
    FOC_InputCmd_t cmd = {0};
    if (driver->read) {
        driver->read(&cmd);
    }
    s_throttle_active = cmd.throttle;

    FOC_State_t state = FOC_GetState();

    /* 4. Safety Arming Interlock (Zero-Throttle Lock) */
    if (state == FOC_STATE_IDLE || state == FOC_STATE_FAULT) {
        if (state == FOC_STATE_FAULT) {
            s_armed = 0;
            s_zero_count = 0;
        }
        if (!s_armed) {
            if (cmd.throttle <= 0.001f) {
                if (++s_zero_count >= 100) { /* 100ms at zero throttle */
                    s_armed = 1;
                }
            } else {
                s_zero_count = 0;
            }
        }
    }

    /* 5. Start Trigger: apply configured input_mode and start motor */
    if (state == FOC_STATE_IDLE && s_armed && cmd.arm_req && cmd.is_active) {
        uint8_t mode_val = (uint8_t)(g_foc.cfg.input_mode + 0.5f);
        if (mode_val > 2) mode_val = 2;
        FOC_SetControlMode((FOC_ControlMode_t)mode_val);
        FOC_Start();
    }

    /* 6. Stop Trigger & Failsafe */
    if (state != FOC_STATE_IDLE && state != FOC_STATE_FAULT && state != FOC_STATE_STOP) {
        if (!cmd.arm_req || !cmd.is_active) {
            FOC_SetVoltageRef(0.0f);
            FOC_SetSpeedRef(0.0f);
            FOC_SetTorqueRef(0.0f);
            FOC_Stop();
            s_armed = 0;
            s_zero_count = 0;
            s_dynamic_min_vq = 0.0f;
        }
    }

    /* 7. Target Dispatcher when Running */
    if (state == FOC_STATE_RUN && cmd.is_active) {
        switch (g_foc.status.control_mode) {
            case FOC_MODE_SPEED: {
                float min_spd = g_foc.cfg.input_min_spd;
                float max_spd = g_foc.cfg.motor_max_spd;
                float target_rpm = min_spd + cmd.throttle * (max_spd - min_spd);
                FOC_SetSpeedRef(target_rpm);
                break;
            }
            case FOC_MODE_TORQUE: {
                float min_cur = g_foc.cfg.input_min_cur;
                float max_cur = g_foc.cfg.motor_max_curr;
                float target_cur = min_cur + cmd.throttle * (max_cur - min_cur);
                float target_pct = (max_cur > 0.001f) ? ((target_cur / max_cur) * 100.0f) : 0.0f;
                FOC_SetTorqueRef(target_pct);
                break;
            }
            case FOC_MODE_VOLTAGE: {
                float min_vq = g_foc.cfg.input_min_vq;
                if (s_dynamic_min_vq > min_vq) {
                    min_vq = s_dynamic_min_vq;
                }
                float target_vq_pct = (min_vq + cmd.throttle * (1.0f - min_vq)) * 100.0f;
                FOC_SetVoltageRef(target_vq_pct);
                break;
            }
            default:
                break;
        }
    }
}
