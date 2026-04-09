/**
 * @file motor_id.c
 * @brief Motor Parameter Identification Implementation
 *
 * Rs: Two-point DC injection at different current levels, linear regression.
 * Ls: Short voltage pulses with integral RL step-response equation and
 *     multi-pulse accumulation for noise rejection.
 */

#include "motor_id.h"

#include <math.h>

#include "foc_config.h"
#include "foc_state_machine.h"

/*===========================================================================*/
/* Internal Constants                                                        */
/*===========================================================================*/
#define ID_DUTY_STEP 0.005f    /* Duty increment per ramp step           */
#define ID_CURRENT_TOL 0.1f    /* Current stability tolerance [A]        */
#define ID_VALID_COUNT 100     /* Consecutive stable samples to confirm  */
#define ID_ALIGN_TIME_MS 100   /* Rotor alignment hold time [ms]         */
#define ID_RS_DELAY_MS 500     /* Settling time before Rs sampling [ms]  */
#define ID_RS_SAMPLES 500      /* Number of Rs averaging samples         */
#define ID_LS_SETTLE_MS 200    /* Decay + settle time before Ls pulse [ms]*/
#define ID_LS_PULSE_SAMPLES 5  /* Samples per Ls voltage pulse           */
#define ID_LS_PULSES 20        /* Number of pulses to accumulate         */
#define ID_LS_PULSE_DUTY 0.15f /* Additional duty during Ls pulse        */

/* Rs measurement current levels (fraction of motor_max_curr) */
#define ID_RS_CURRENT_LO 0.05f
#define ID_RS_CURRENT_HI 0.15f

/* Align current (fraction of motor_max_curr) */
#define ID_ALIGN_CURRENT 0.10f

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

MotorID_Result_t id_result;

static uint32_t id_timer_ms;
static uint32_t id_sample_count;
static uint32_t id_count_valid;
static uint32_t tick_counter;

static float id_current_filt;
static float id_current_filt_prev;
static uint8_t ramp_first_run;

/* Rs: two-point measurement accumulators */
static float rs_sum_current[2];
static float rs_sum_voltage[2];
static int rs_point_idx;

/* Ls: integral approach on short voltage pulses */
static int ls_phase;
static int ls_pulse_count;
static int ls_sample_idx;
static float ls_start_current;
static float ls_prev_current;
static float ls_stop_threshold_A;
/* Accumulators for exact RL step response integration across pulses:
 *   L = (Σ V*dt - Rs * Σ∫(I-I0)dt) / Σ(I_end - I_start)              */
static float ls_sum_V_dt;
static float ls_sum_I_diff;
static float ls_sum_R_Idt;

/*===========================================================================*/
/* Private Helpers                                                           */
/*===========================================================================*/

/** Enable phases A+B high-side, C off — standard 2-phase topology for ID. */
static inline void id_enable_ab(void) {
    FOC_EnableDriver(1, 1);
    FOC_EnableDriver(2, 1);
    FOC_EnableDriver(3, 1);
}

/** Reset Ls integral accumulators. */
static void ls_reset_accumulators(void) {
    ls_sum_V_dt = 0.0f;
    ls_sum_I_diff = 0.0f;
    ls_sum_R_Idt = 0.0f;
}

/**
 * Slowly ramp duty until the measured current reaches target_current.
 * Uses a simple bang-bang with hysteresis: only adjusts duty once the
 * filtered current has been stable for ID_VALID_COUNT consecutive samples.
 */
static void ramp_current_duty(float* duty_a, float* duty_b, float* duty_c, float current,
                              float target_current) {
    if (ramp_first_run) {
        *duty_a = ID_DUTY_STEP;
        *duty_b = 0.0f;
        *duty_c = 0.0f;
        ramp_first_run = 0;
        return;
    }

    id_current_filt =
        id_current_filt * (1.0f - CURRENT_FILTER_COEFF) + current * CURRENT_FILTER_COEFF;

    float delta = fabsf(id_current_filt - id_current_filt_prev);
    id_count_valid = (delta < ID_CURRENT_TOL) ? (id_count_valid + 1) : 0;

    if (id_count_valid >= ID_VALID_COUNT) {
        float err = target_current - id_current_filt;
        if (fabsf(err) < ID_CURRENT_TOL) {
            id_current_filt_prev = id_current_filt;
            return;
        }
        *duty_a += (err > 0.0f) ? ID_DUTY_STEP : -ID_DUTY_STEP;
        *duty_b = 0.0f;
        *duty_c = 0.0f;
        id_count_valid = 0;
    }

    id_current_filt_prev = id_current_filt;
}

/** Reset the ramp state for a fresh current target. */
static void reset_ramp(void) {
    ramp_first_run = 1;
    id_current_filt = 0.0f;
    id_current_filt_prev = 0.0f;
    id_count_valid = 0;
}

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

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
    rs_point_idx = 0;

    rs_sum_current[0] = 0.0f;
    rs_sum_current[1] = 0.0f;
    rs_sum_voltage[0] = 0.0f;
    rs_sum_voltage[1] = 0.0f;

    ls_phase = 0;
    ls_pulse_count = 0;
    ls_sample_idx = 0;
    ls_start_current = 0.0f;

    reset_ramp();
    ls_reset_accumulators();
}

void MotorID_Stop(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
}

void MotorID_RunStep(float ia, float ib, float ic, float vbus, float* duty_a, float* duty_b,
                     float* duty_c) {
    /* ---- Millisecond timer from PWM ticks ---- */
    if (++tick_counter >= (PWM_FREQUENCY / 1000)) {
        id_timer_ms++;
        tick_counter = 0;
    }

    float current = (ia - (ib + ic) / 2);
    float Vdeadtime = 0.0f;
    /* ---- Enable/disable drivers on state transitions ---- */
    static MotorID_State_t last_state = MOTOR_ID_STATE_IDLE;
    if (id_result.state != last_state) {
        switch (id_result.state) {
            case MOTOR_ID_STATE_ALIGN:
            case MOTOR_ID_STATE_MEASURE_RS:
            case MOTOR_ID_STATE_MEASURE_LS:
                id_enable_ab();
                break;
            default:
                FOC_EnableDrivers(0);
                break;
        }
        last_state = id_result.state;
    }

    /* ---- Main state machine ---- */
    switch (id_result.state) {
        case MOTOR_ID_STATE_IDLE:
        case MOTOR_ID_STATE_COMPLETE:
            break;

        case MOTOR_ID_STATE_ERROR:
            FOC_EnableDrivers(0);
            break;

        /* ============================================================== */
        /* ALIGN: Lock rotor to a known electrical angle                   */
        /* ============================================================== */
        case MOTOR_ID_STATE_ALIGN:
            ramp_current_duty(duty_a, duty_b, duty_c, current,
                              g_foc.cfg.motor_max_curr * ID_ALIGN_CURRENT);
            if (id_timer_ms >= ID_ALIGN_TIME_MS) {
                id_result.state = MOTOR_ID_STATE_MEASURE_RS;
                id_timer_ms = 0;
                reset_ramp();
            }
            break;

        /* ============================================================== */
        /* Rs: Two-point DC measurement                                    */
        /*   Point 0: low current  → V₁, I₁                               */
        /*   Point 1: high current → V₂, I₂                               */
        /*   Rs = (V₂-V₁) / (I₂-I₁) / 2   (÷2 for two phases in series)  */
        /* ============================================================== */
        case MOTOR_ID_STATE_MEASURE_RS: {
            float target = (rs_point_idx == 0) ? g_foc.cfg.motor_max_curr * ID_RS_CURRENT_LO
                                               : g_foc.cfg.motor_max_curr * ID_RS_CURRENT_HI;

            /* Ramp to target and let it settle */
            if (id_timer_ms < ID_RS_DELAY_MS) {
                ramp_current_duty(duty_a, duty_b, duty_c, current, target);
            }

            /* Accumulate samples after settling */
            if (id_timer_ms >= ID_RS_DELAY_MS) {
                rs_sum_current[rs_point_idx] += current;
                rs_sum_voltage[rs_point_idx] += vbus * (*duty_a);
                id_sample_count++;
            }

            /* Move to next point or compute result */
            if (id_sample_count >= ID_RS_SAMPLES) {
                if (rs_point_idx == 0) {
                    rs_point_idx = 1;
                    id_timer_ms = 0;
                    id_sample_count = 0;
                    reset_ramp();
                } else {
                    float inv = 1.0f / (float)ID_RS_SAMPLES;
                    float I1 = rs_sum_current[0] * inv;
                    float I2 = rs_sum_current[1] * inv;
                    float V1 = rs_sum_voltage[0] * inv;
                    float V2 = rs_sum_voltage[1] * inv;
                    float dI = I2 - I1;

                    if (dI < 0.01f) {
                        id_result.error_code = 1;
                        id_result.state = MOTOR_ID_STATE_ERROR;
                        break;
                    }

                    id_result.measured_rs = (V2 - V1) / dI * (2.0f / 3.0f);
                    id_result.state = MOTOR_ID_STATE_MEASURE_LS;
                    id_timer_ms = 0;
                    ls_phase = 0;
                    ls_pulse_count = 0;
                    float v_dt_1 = V1 - I1 * (1.5 * id_result.measured_rs);
                    float v_dt_2 = V2 - I2 * (1.5 * id_result.measured_rs);
                    Vdeadtime = (v_dt_1 + v_dt_2) * 0.5;
                    ls_reset_accumulators();
                }
            }
        } break;

        /* ============================================================== */
        /* Ls: Integral method on short voltage pulses                     */
        /*                                                                 */
        /* Uses exact RL step response equation:                           */
        /*   L = (V·T - Rs·∫(I-I₀)dt) / (I_end - I_start)                */
        /*                                                                 */
        /* Sub-phases:                                                     */
        /*   0: Settle — set duty=0 and wait for current to decay          */
        /*   1: Pulse  — apply step voltage and accumulate integrals       */
        /*   2: Eval   — compute Ls from accumulated data                  */
        /* ============================================================== */
        case MOTOR_ID_STATE_MEASURE_LS: {
            switch (ls_phase) {
                /* ---- Phase 0: Wait for current to decay to ~0 ---- */
                case 0:
                    *duty_a = 0.0f;
                    *duty_b = 0.0f;
                    *duty_c = 0.0f;
                    if (id_timer_ms >= ID_LS_SETTLE_MS) {
                        ls_phase = 1;
                        ls_sample_idx = 0;
                        id_timer_ms = 0;
                        id_enable_ab();
                        float v_pulse_actual = (vbus * ID_LS_PULSE_DUTY) - Vdeadtime;
                        float r_eq = 1.5f * id_result.measured_rs;
                        float i_max_theoretical = v_pulse_actual / r_eq;
                        ls_stop_threshold_A = i_max_theoretical * 0.30f;
                    }
                    break;

                /* ---- Phase 1: Voltage pulse & integral accumulation ---- */
                case 1:
                    *duty_a = ID_LS_PULSE_DUTY;
                    *duty_b = 0.0f;
                    *duty_c = 0.0f;

                    if (ls_sample_idx == 0) {
                        ls_start_current = current;
                        ls_prev_current = current;
                    } else {
                        /* Trapezoidal sum: Rs · ∫(I - I₀)dt */
                        float i_delta_avg = ((current + ls_prev_current) / 2.0f) - ls_start_current;
                        ls_sum_R_Idt +=
                            (1.5f * id_result.measured_rs) * i_delta_avg * CONTROL_PERIOD_F;
                        ls_prev_current = current;
                    }
                    float current_diff = current - ls_start_current;
                    uint8_t stop_condition =
                        (ls_sample_idx >= ID_LS_PULSE_SAMPLES - 1) ||
                        (ls_sample_idx > 1 && current_diff > ls_stop_threshold_A);
                    if (stop_condition) {
                        /* Accumulate pulse results */
                        ls_sum_V_dt += (vbus * ID_LS_PULSE_DUTY - Vdeadtime) * ls_sample_idx *
                                       CONTROL_PERIOD_F;
                        ls_sum_I_diff += current_diff;
                        ls_pulse_count++;
                        if (ls_pulse_count >= ID_LS_PULSES) {
                            ls_phase = 2;
                        } else {
                            ls_phase = 0;
                        }
                        id_timer_ms = 0;
                    } else {
                        ls_sample_idx++;
                    }
                    break;

                /* ---- Phase 2: Compute Ls from accumulated data ---- */
                case 2:
                    if (ls_sum_I_diff > 0.001f) {
                        id_result.measured_ls = (ls_sum_V_dt - ls_sum_R_Idt) / ls_sum_I_diff;
                    } else {
                        id_result.error_code = 2;
                    }
                    id_result.state = MOTOR_ID_STATE_COMPLETE;
                    break;
            }
        } break;
    }
}

void MotorID_GetResults(MotorID_Result_t* results) {
    if (results) {
        *results = id_result;
    }
}
