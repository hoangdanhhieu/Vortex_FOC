#include "foc_input.h"

#include <stddef.h>

#include "foc_state_machine.h"
#include "input_dshot.h"
#include "input_pot.h"
#include "motor_id.h"

static FOC_InputSource_t s_input_source = FOC_INPUT_SOURCE_NONE;
static const FOC_InputDriver_t* s_active_driver = NULL;

static float s_throttle_active = 0.0f;
static float s_dynamic_min_vq = 0.0f;
static uint8_t s_armed = 0;
static uint8_t s_prev_input_active = 0;
static uint8_t s_prev_armed_ready = 0;
static uint16_t s_fault_clear_count = 0;
static volatile uint16_t s_host_override_ticks = 0;
static volatile uint8_t s_host_controlling = 0;

void FOC_Input_SelectSource(FOC_InputSource_t source) {
    if (source > FOC_INPUT_SOURCE_DSHOT) {
        source = FOC_INPUT_SOURCE_NONE;
    }

    if (s_active_driver && s_active_driver->deinit) {
        s_active_driver->deinit();
    }

    s_input_source = source;
    s_armed = 0;
    s_prev_input_active = 0;
    s_prev_armed_ready = 0;
    s_fault_clear_count = 0;
    s_throttle_active = 0.0f;

    switch (source) {
        case FOC_INPUT_SOURCE_POT:
            s_active_driver = &g_driver_pot;
            break;
        case FOC_INPUT_SOURCE_DSHOT:
            s_active_driver = &g_driver_dshot;
            break;
        default:
            s_active_driver = NULL;
            break;
    }

    if (s_active_driver && s_active_driver->init) {
        s_active_driver->init();
    }
}

void FOC_Input_Init(void) {
    s_dynamic_min_vq = 0.0f;
    s_host_override_ticks = 0;
    s_host_controlling = 0;

    uint8_t src_cfg = (uint8_t)(g_foc.cfg.input_source + 0.5f);
    FOC_Input_SelectSource((FOC_InputSource_t)src_cfg);
}

void FOC_Input_SetMinVq(float vq_min) {
    s_dynamic_min_vq = vq_min;
}

void FOC_Input_NotifyHostActivity(uint16_t timeout_ms) {
    if (timeout_ms > 0) {
        s_host_override_ticks = timeout_ms;
        s_host_controlling = 1;
    } else {
        s_host_override_ticks = 0;
        if (s_host_controlling) {
            s_host_controlling = 0;
            s_armed = 0;
            s_throttle_active = 0.0f;
        }
    }
}

uint8_t FOC_Input_IsHostControlling(void) {
    return s_host_controlling;
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

static void FOC_Input_MapThrottle(float thr) {
    float sign = (thr < 0.0f) ? -1.0f : 1.0f;
    thr = fabsf(thr);
    switch (g_foc.status.control_mode) {
        case FOC_MODE_SPEED: {
            float min_spd = g_foc.cfg.input_min_spd;
            float max_spd = g_foc.cfg.motor_max_spd;
            float spd = min_spd + thr * (max_spd - min_spd);
            FOC_SetSpeedRef(sign * spd);
            break;
        }
        case FOC_MODE_TORQUE: {
            float min_cur = g_foc.cfg.input_min_cur;
            float max_cur = g_foc.cfg.motor_max_curr;
            float cur = min_cur + thr * (max_cur - min_cur);
            FOC_SetTorqueCurrent(sign * cur);
            break;
        }
        case FOC_MODE_VOLTAGE: {
            float min_vq = g_foc.cfg.input_min_vq;
            if (s_dynamic_min_vq > min_vq) {
                min_vq = s_dynamic_min_vq;
            }
            float vq = min_vq + thr * (1.0f - min_vq);
            FOC_SetVoltageRef(sign * vq);
            break;
        }
        default:
            break;
    }
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

void FOC_Input_Update(void) {
    if (FOC_IsProfilerActive() || MotorID_IsRunning()) {
        return;
    }

    if (s_host_controlling) {
        if (s_host_override_ticks > 0) {
            s_host_override_ticks--;
        } else {
            s_host_controlling = 0;
            FOC_State_t state = g_foc.status.state;
            if (state != FOC_STATE_IDLE && state != FOC_STATE_STOP && state != FOC_STATE_FAULT) {
                FOC_Input_DoStop();
            }
        }
        return;
    }

    uint8_t src_cfg = (uint8_t)(g_foc.cfg.input_source + 0.5f);
    if (src_cfg != (uint8_t)s_input_source) {
        FOC_Input_SelectSource((FOC_InputSource_t)src_cfg);
    }
    if (s_active_driver == NULL) {
        return;
    }

    FOC_InputCmd_t cmd = {0};
    if (s_active_driver->update) {
        s_active_driver->update(&cmd);
    }
    if (s_active_driver->read_throttle) {
        s_throttle_active = s_active_driver->read_throttle();
    }

    if (g_foc.status.state == FOC_STATE_FAULT) {
        if (!cmd.arm_req && cmd.is_active) {
            if (++s_fault_clear_count >= 200U) {
                FOC_ClearFault();
                s_fault_clear_count = 0;
            }
        } else {
            s_fault_clear_count = 0;
        }
        s_armed = 0;
        return;
    }

    s_fault_clear_count = 0;
    s_armed = (cmd.is_active && cmd.arm_req) ? 1U : 0U;
    FOC_State_t state = g_foc.status.state;

    uint8_t active_rise = (cmd.is_active && !s_prev_input_active) ? 1U : 0U;
    uint8_t armed_rise = (cmd.is_armed_ready && !s_prev_armed_ready) ? 1U : 0U;
    s_prev_input_active = cmd.is_active;
    s_prev_armed_ready = cmd.is_armed_ready;

    uint8_t has_throttle = (fabsf(s_throttle_active) > 1e-4f) ? 1U : 0U;

    if ((state == FOC_STATE_IDLE || (state == FOC_STATE_BEEP && has_throttle)) && s_armed) {
        uint8_t mode_val = (uint8_t)(g_foc.cfg.input_mode + 0.5f);
        if (mode_val > 2) mode_val = 2;
        if (FOC_SetControlMode((FOC_ControlMode_t)mode_val)) {
            FOC_Input_MapThrottle(s_throttle_active);
            FOC_Start();
        }
    } else if (state == FOC_STATE_IDLE && !s_armed) {
        if (cmd.beacon_req >= 1U && cmd.beacon_req <= 5U) {
            static const FOC_BeepType_t k_beacon_types[5] = {
                FOC_BEEP_BEACON_1, FOC_BEEP_BEACON_2, FOC_BEEP_BEACON_3,
                FOC_BEEP_BEACON_4, FOC_BEEP_BEACON_5,
            };
            FOC_PlayBeep(k_beacon_types[cmd.beacon_req - 1U]);
        } else if (armed_rise) {
            FOC_PlayBeep(FOC_BEEP_ARMED);
        } else if (active_rise && s_input_source == FOC_INPUT_SOURCE_DSHOT) {
            FOC_PlayBeep(FOC_BEEP_SIGNAL_LOCK);
        }
    } else if (state != FOC_STATE_IDLE && state != FOC_STATE_FAULT && state != FOC_STATE_STOP &&
               state != FOC_STATE_BEEP && !s_armed) {
        FOC_Input_DoStop();
    } else if (s_armed && s_input_source == FOC_INPUT_SOURCE_POT) {
        FOC_Input_MapThrottle(s_throttle_active);
    }
}

void FOC_Input_PushThrottle(float thr) {
    if (s_host_controlling || s_host_override_ticks > 0 || FOC_IsProfilerActive() ||
        MotorID_IsRunning() || !s_armed) {
        return;
    }
    s_throttle_active = thr;
    FOC_Input_MapThrottle(thr);
}