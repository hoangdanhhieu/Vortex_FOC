/**
 * @file motor_id.c
 * @brief Motor Parameter Identification Implementation
 *
 * Ls measurement uses short voltage pulses (starting from ~zero current)
 * with online least squares regression and two-direction dead-time
 * cancellation.  Short pulses prevent core saturation.
 */

#include "motor_id.h"

#include <math.h>

#include "foc_config.h"
#include "foc_state_machine.h"
/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

MotorID_Result_t id_result;
static uint32_t id_timer_ms;
static uint32_t id_sample_count;
static uint32_t id_count_valid_current;
static float id_current_filter;
static float id_current_filter_prev;
static float measure_current[2];
static float measure_voltage[2];
static int measure_th;
static uint32_t tick_counter = 0;
static uint8_t first_run;

/* Ls measurement: Integral approach on short pulses */
static int ls_phase;           /* Sub-state machine */
static int ls_pulse_count;     /* Completed pulses in current direction */
static int ls_sample_idx;      /* Sample index within current pulse */
static float ls_start_current; /* I[0] for current pulse */
static float ls_base_duty_a;
static float ls_base_duty_b;
static float ls_base_duty_c;

static int ls_sweep_idx; /* Current index for Ls saturation LUT generation */

/* Accumulators for exact RL step response integration across pulses */
static float ls_sum_V_dt;   /* Σ (V_pulse * dT) */
static float ls_sum_I_diff; /* Σ (I_end - I_start) */
static float ls_sum_R_Idt;  /* Σ (Rs * ∫(I - I_start)dt) */

/*===========================================================================*/
/* Private Helpers                                                           */
/*===========================================================================*/

static void ls_reset_accumulators(void) {
    ls_sum_V_dt = 0.0f;
    ls_sum_I_diff = 0.0f;
    ls_sum_R_Idt = 0.0f;
}

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

void ramp_current_duty(float* duty_a, float* duty_b, float* duty_c, float current,
                       float target_current) {
    if (first_run == 1) {
        *duty_a = g_foc.cfg.id_duty_step;
        *duty_b = 0.0f;
        *duty_c = 0.0f;
        first_run = 0;
        return;
    }
    id_current_filter =
        (id_current_filter * (1 - CURRENT_FILTER_COEFF)) + (current * CURRENT_FILTER_COEFF);
    float delta_current = fabs(id_current_filter - id_current_filter_prev);
    if (delta_current < g_foc.cfg.id_current_tol) {
        id_count_valid_current++;
    } else {
        id_count_valid_current = 0;
    }
    if (id_count_valid_current >= (uint32_t)g_foc.cfg.id_valid_count) {
        float err = target_current - id_current_filter;
        if (err < g_foc.cfg.id_current_tol) {
            return;
        }
        if (err > 0)
            *duty_a += g_foc.cfg.id_duty_step;
        else
            *duty_a -= g_foc.cfg.id_duty_step;
        *duty_b = 0.0f;
        *duty_c = 0.0f;
        id_count_valid_current = 0;
    }
    id_current_filter_prev = id_current_filter;
}

void reset_ramp(void) {
    first_run = 1;
    id_current_filter = 0;
    id_current_filter_prev = 0;
    id_count_valid_current = 0;
}

void MotorID_Init(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
    id_result.measured_rs = 0.0f;
    id_result.measured_ls = 0.0f;
    id_result.error_code = 0;
}

void MotorID_Start(void) {
    id_result.state = MOTOR_ID_STATE_ALIGN;
    id_timer_ms = 0;
    id_sample_count = 0;
    tick_counter = 0;
    measure_th = 0;
    id_count_valid_current = 0;
    id_current_filter = 0.0f;
    measure_current[0] = 0.0f;
    measure_current[1] = 0.0f;
    measure_voltage[0] = 0.0f;
    measure_voltage[1] = 0.0f;
    first_run = 1;
    /* Ls variables */
    ls_phase = 0;
    ls_pulse_count = 0;
    ls_sample_idx = 0;
    ls_start_current = 0.0f;
    ls_sweep_idx = 0;
    id_result.ls_lut_count = 0;
    ls_reset_accumulators();
    g_foc.status.state = FOC_STATE_CALIBRATION;
}

void MotorID_Stop(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
}

void MotorID_RunStep(float ia, float ib, float vbus, float* duty_a, float* duty_b, float* duty_c) {
    tick_counter++;

    if (tick_counter >= (PWM_FREQUENCY / 1000)) {
        id_timer_ms++;
        tick_counter = 0;
    }
    float current = (ia - ib) * 0.5f;

    static MotorID_State_t last_state = MOTOR_ID_STATE_IDLE;
    if (id_result.state != last_state) {
        /* Update drivers only on state change to save CPU cycles */
        switch (id_result.state) {
            case MOTOR_ID_STATE_ALIGN:
            case MOTOR_ID_STATE_MEASURE_RS:
                FOC_EnableDriver(1, 1);
                FOC_EnableDriver(2, 1);
                FOC_EnableDriver(3, 0);
                break;
            case MOTOR_ID_STATE_MEASURE_LS:
                /* LS handled state-by-state due to pulse nature */
                break;
            default:
                FOC_EnableDrivers(0);
                break;
        }
        last_state = id_result.state;
    }

    switch (id_result.state) {
        case MOTOR_ID_STATE_IDLE:
        case MOTOR_ID_STATE_COMPLETE:
            break;
        case MOTOR_ID_STATE_ERROR:
            FOC_EnableDrivers(0);
            break;
        case MOTOR_ID_STATE_ALIGN:
            FOC_EnableDriver(1, 1);
            FOC_EnableDriver(2, 1);
            FOC_EnableDriver(3, 0);
            ramp_current_duty(duty_a, duty_b, duty_c, current, g_foc.cfg.id_align_current);
            if (id_timer_ms >= (uint32_t)g_foc.cfg.id_align_time_ms) {
                id_result.state = MOTOR_ID_STATE_MEASURE_RS;
                id_timer_ms = 0;
                reset_ramp();
            }
            break;

        case MOTOR_ID_STATE_MEASURE_RS: {
            FOC_EnableDriver(1, 1);
            FOC_EnableDriver(2, 1);
            FOC_EnableDriver(3, 0);
            if (id_timer_ms < (uint32_t)g_foc.cfg.id_rs_delay_ms) {
                if (measure_th == 0) {
                    ramp_current_duty(duty_a, duty_b, duty_c, current, g_foc.cfg.id_rs_current1);
                } else {
                    ramp_current_duty(duty_a, duty_b, duty_c, current, g_foc.cfg.id_rs_current2);
                }
            }
            if (id_timer_ms >= (uint32_t)g_foc.cfg.id_rs_delay_ms) {
                measure_current[measure_th] += current;
                measure_voltage[measure_th] += vbus * (*duty_a);
                id_sample_count++;
            }
            if (id_sample_count >= (uint32_t)g_foc.cfg.id_rs_samples) {
                if (measure_th == 0) {
                    measure_th = 1;
                    id_timer_ms = 0;
                    id_sample_count = 0;
                    reset_ramp();
                } else {
                    float inv_samples = 1.0f / (float)g_foc.cfg.id_rs_samples;
                    float I_1_avg = measure_current[0] * inv_samples;
                    float I_2_avg = measure_current[1] * inv_samples;
                    float V_1_avg = measure_voltage[0] * inv_samples;
                    float V_2_avg = measure_voltage[1] * inv_samples;

                    float dI = I_2_avg - I_1_avg;
                    if (dI < 0.01f) {
                        id_result.error_code = 1;
                        id_result.state = MOTOR_ID_STATE_ERROR;
                        break;
                    }
                    id_result.measured_rs = (V_2_avg - V_1_avg) / dI * 0.5f;

                    id_result.state = MOTOR_ID_STATE_MEASURE_LS;
                    id_timer_ms = 0;
                    ls_phase = 0;
                    ls_pulse_count = 0;
                    ls_sweep_idx = 0;
                    id_result.ls_lut_count = 0;
                    reset_ramp();
                    ls_reset_accumulators();
                }
            }
        } break;

        /*=================================================================*/
        /* Ls MEASUREMENT: Integral logic on short voltage pulses          */
        /*                                                                 */
        /* Uses exact physical equation for an RL circuit step response:      */
        /* L = (V_pulse * T - Rs * Integral(I - I_start)dt) / (I_end - I_start) */
        /* Eliminates the catastrophic cancellation in float logic of OLS  */
        /* and requires no logarithm math. Tolerates zero bias currents too. */
        /*                                                                 */
        /* Accumulates results across MOTOR_ID_LS_NUM_PULSES.              */
        /*                                                                 */
        /* Sub-phases:                                                     */
        /*   0: Decay (drivers off, I→0)                                   */
        /*   1: Constant duty tracking                                     */
        /*   2: Pulse and evaluate step integral                           */
        /*   3: Evaluate result and complete                               */
        /*=================================================================*/
        case MOTOR_ID_STATE_MEASURE_LS: {
            float current_target = 0.0f;
            // if (MOTOR_ID_LS_IBIAS_STEPS > 1) {
            //     float delta_I = (MOTOR_ID_LS_IBIAS_MAX - MOTOR_ID_LS_IBIAS_MIN) /
            //                     (float)(MOTOR_ID_LS_IBIAS_STEPS - 1);
            //     current_target = MOTOR_ID_LS_IBIAS_MIN + (float)ls_sweep_idx * delta_I;
            // } else {
            //     current_target = MOTOR_ID_LS_IBIAS_MIN;
            // }
            switch (ls_phase) {
                case 0:
                    ramp_current_duty(duty_a, duty_b, duty_c, current, 0.0f);
                    if (id_timer_ms >= ((uint32_t)g_foc.cfg.id_ls_decay_ms / 10 + 1)) {
                        ls_phase = 1;
                        ls_sample_idx = 0;
                        id_timer_ms = 0;
                    }
                    break;
                case 1:
                    ramp_current_duty(duty_a, duty_b, duty_c, current, current_target);
                    ls_base_duty_a = *duty_a;
                    ls_base_duty_b = *duty_b;
                    ls_base_duty_c = *duty_c;
                    if (id_timer_ms >= (uint32_t)g_foc.cfg.id_ls_delay_ms) {
                        ls_phase = 2;
                        ls_sample_idx = 0;
                        id_timer_ms = 0;
                        /* Re-enable drivers specifically for pulse */
                        FOC_EnableDriver(1, 1);
                        FOC_EnableDriver(2, 1);
                        FOC_EnableDriver(3, 0);
                    }
                    break;
                case 2:
                    *duty_a = ls_base_duty_a + g_foc.cfg.id_ls_pulse_v / vbus;
                    *duty_b = ls_base_duty_b;
                    *duty_c = ls_base_duty_c;

                    if (ls_sample_idx == 0) {
                        ls_start_current = current;
                    } else {
                        /* Riemann sum for Rs voltage drop area: ∫ (I - I_start) dt */
                        ls_sum_R_Idt +=
                            id_result.measured_rs * (current - ls_start_current) * CONTROL_PERIOD_F;
                    }

                    if (ls_sample_idx >= (int)g_foc.cfg.id_ls_pulse_samples - 1) {
                        /* Final sample of pulse */
                        ls_sum_V_dt += g_foc.cfg.id_ls_pulse_v *
                                       ((float)g_foc.cfg.id_ls_pulse_samples - 1.0f) *
                                       CONTROL_PERIOD_F;
                        ls_sum_I_diff += (current - ls_start_current);

                        ls_pulse_count++;
                        if (ls_pulse_count >= (int)g_foc.cfg.id_ls_pulses) {
                            ls_phase = 3;
                        } else {
                            ls_phase = 0;
                            reset_ramp();
                        }
                        id_timer_ms = 0;
                    } else {
                        ls_sample_idx++;
                    }
                    break;
                case 3: {
                    float measured_ls = 0.0f;
                    if (ls_sum_I_diff > 0.001f) {
                        measured_ls = (ls_sum_V_dt - ls_sum_R_Idt) / ls_sum_I_diff;
                    } else {
                        id_result.error_code = 1;
                    }

                    if (ls_sweep_idx == 0) {
                        id_result.measured_ls = measured_ls;
                    }

                    if (ls_sweep_idx < MOTOR_ID_MAX_LUT_SIZE && id_result.error_code == 0) {
                        id_result.ls_lut_currents[ls_sweep_idx] = current_target;
                        id_result.ls_lut_values[ls_sweep_idx] = measured_ls;
                        id_result.ls_lut_count++;
                    }

                    ls_sweep_idx++;
                    if (ls_sweep_idx >= 1 || id_result.error_code != 0) {
                        id_result.state = MOTOR_ID_STATE_COMPLETE;
                    } else {
                        ls_phase = 0;
                        id_timer_ms = 0;
                        ls_pulse_count = 0;
                        ls_reset_accumulators();
                        reset_ramp();
                    }
                } break;
            }
        } break;
    }
}

void MotorID_GetResults(MotorID_Result_t* results) {
    if (results) {
        *results = id_result;
    }
}
