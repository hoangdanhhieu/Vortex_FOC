/**
 * @file foc_slow_task.c
 * @brief FOC 1kHz Slow Task, Speed Loop, Ramps & Safety Protections Implementation
 */

#include "foc_slow_task.h"

#include "foc.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "ladrc_controller.h"
#include "math.h"
#include "peripheral_init.h"
#include "foc_input.h"

/* Safety check state */
static uint32_t stall_counter = 0;      /* Stall timer (ticks) */
static uint32_t ground_fault_count = 0; /* Ground fault deglitch counter */

extern volatile float ADC_Vref;

void FOC_SlowTask(void) {
    if (FOC_GetState() == FOC_STATE_IDLE && !FOC_IsInitialized()) return;
    if (!FOC_IsInitialized()) return;

    /* --- Cập nhật và xử lý biến trở từ mô-đun input --- */
    FOC_Input_Update();

    /* --- Vbus measurement at 1 kHz --- */
    uint16_t vbus_raw = ADC_ReadVbus_SingleShot();
    if (vbus_raw != 0) {
        float vref = ADC_Vref;
        if (vref < 1.0f) vref = 3.3f;
        float v_scale = vref * (VBUS_DIVIDER_RATIO / 4096.0f);
        float vbus_new = (float)vbus_raw * v_scale;

        /* IIR Low-Pass Filter */
        g_foc.data.Vbus = VBUS_IIR_ALPHA * g_foc.data.Vbus + (1.0f - VBUS_IIR_ALPHA) * vbus_new;
        if (g_foc.data.Vbus < 1.0f) g_foc.data.Vbus = 1.0f;
        g_foc.data.Vbus_inv = 1.0f / g_foc.data.Vbus;
    }

    /* Software OV/UV protection (replaces hardware AWD2) */
    if (g_foc.status.state == FOC_STATE_RUN || g_foc.status.state == FOC_STATE_STARTUP) {
        if (g_foc.data.Vbus > g_foc.cfg.fault_ov_threshold) {
            FOC_EnableDrivers(0);
            g_foc.status.fault = FOC_FAULT_OVERVOLTAGE;
            g_foc.status.state = FOC_STATE_FAULT;
        } else if (g_foc.data.Vbus < g_foc.cfg.fault_uv_threshold) {
            FOC_EnableDrivers(0);
            g_foc.status.fault = FOC_FAULT_UNDERVOLTAGE;
            g_foc.status.state = FOC_STATE_FAULT;
        }
    }

    if (g_foc.status.state == FOC_STATE_RUN) {
        if (g_foc.data.duty_a < 0.85f && g_foc.data.duty_b < 0.85f && g_foc.data.duty_c < 0.85f &&
            g_foc.data.duty_a > 0.15f && g_foc.data.duty_b > 0.15f && g_foc.data.duty_c > 0.15f) {
            float current_sum = g_foc.data.Ia + g_foc.data.Ib + g_foc.data.Ic;
            float gf_threshold = 0.40f * g_foc.cfg.motor_max_curr;
            if (gf_threshold < 1.0f) gf_threshold = 1.0f;

            if (fabsf(current_sum) > gf_threshold) {
                if (++ground_fault_count >= 2) {
                    FOC_EnableDrivers(0);
                    g_foc.status.fault = FOC_FAULT_GROUND;
                    g_foc.status.state = FOC_STATE_FAULT;
                }
            } else {
                if (ground_fault_count > 0) ground_fault_count--;
            }
        } else {
            if (ground_fault_count > 0) ground_fault_count--;
        }

        if (g_foc.cfg.fault_stall_enable) {
            float threshold_c = g_foc.cfg.fault_stall_current;
            float threshold_c_sq = threshold_c * threshold_c;

            if (g_foc.ctrl.smo.current_err_sq > threshold_c_sq) {
                stall_counter++;
                if (stall_counter >= g_foc.cfg.fault_stall_time_ms) {
                    FOC_EnableDrivers(0);
                    g_foc.status.fault = FOC_FAULT_STALL;
                    g_foc.status.state = FOC_STATE_FAULT;
                }
            } else {
                if (stall_counter > 0) stall_counter--;
            }
        } else {
            stall_counter = 0;
        }
    } else {
        ground_fault_count = 0;
        stall_counter = 0;
    }

    if (g_foc.status.state == FOC_STATE_RUN) {
        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            float accel_rate =
                g_foc.cfg.speed_ramp_accel * RPM_TO_RAD * g_foc.cfg.motor_poles * 0.001f;
            float decel_rate =
                g_foc.cfg.speed_ramp_decel * RPM_TO_RAD * g_foc.cfg.motor_poles * 0.001f;
            float ramp_error = g_foc.cmd.speed_ref_target - g_foc.cmd.speed_ref;

            if (ramp_error > accel_rate) {
                g_foc.cmd.speed_ref += accel_rate;
            } else if (ramp_error < -decel_rate) {
                g_foc.cmd.speed_ref -= decel_rate;
            } else {
                g_foc.cmd.speed_ref = g_foc.cmd.speed_ref_target;
            }

            float target_iq =
                LADRC_Update(&g_foc.ctrl.speed, g_foc.cmd.speed_ref, g_foc.data.omega_elec);

            /* In Speed Mode, LADRC must be allowed to apply torque instantly to track the speed ramp.
             * Applying current_ramp_rate here would cause severe actuator delay windup and triangle oscillations. */
            g_foc.cmd.Iq_ref = target_iq;

            /* Anti-windup tracking for LESO */
            LADRC_SetActualOutput(&g_foc.ctrl.speed, g_foc.cmd.Iq_ref);
        } else if (g_foc.status.control_mode == FOC_MODE_TORQUE) {
            float current_ramp_step = g_foc.cfg.current_ramp_rate * 0.001f;
            float ramp_error = g_foc.cmd.Iq_ref_target - g_foc.cmd.Iq_ref;
            if (ramp_error > current_ramp_step) {
                g_foc.cmd.Iq_ref += current_ramp_step;
            } else if (ramp_error < -current_ramp_step) {
                g_foc.cmd.Iq_ref -= current_ramp_step;
            } else {
                g_foc.cmd.Iq_ref = g_foc.cmd.Iq_ref_target;
            }
        } else if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            float max_v = SQRT3_INV * g_foc.data.Vbus;
            float volt_ramp_step = 0.0f;
            if (max_v > 1.0f) {
                volt_ramp_step = (g_foc.cfg.voltage_ramp_rate / max_v) * 0.001f;
            }
            float ramp_error = g_foc.cmd.Vq_ref_target - g_foc.cmd.Vq_ref;
            if (ramp_error > volt_ramp_step) {
                g_foc.cmd.Vq_ref += volt_ramp_step;
            } else if (ramp_error < -volt_ramp_step) {
                g_foc.cmd.Vq_ref -= volt_ramp_step;
            } else {
                g_foc.cmd.Vq_ref = g_foc.cmd.Vq_ref_target;
            }
        }
    }

    if (g_foc.status.state == FOC_STATE_FAULT || g_foc.status.control_mode != FOC_MODE_VOLTAGE) {
        g_foc.cmd.Vq_ref = 0.0f;
        g_foc.cmd.Vq_ref_target = 0.0f;
    }
    if (g_foc.status.state == FOC_STATE_IDLE) {
        g_foc.cmd.Vq_ref = 0.0f;
    }
}
