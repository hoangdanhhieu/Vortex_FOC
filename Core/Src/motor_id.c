/**
 * @file motor_id.c
 * @brief Motor Parameter Identification — Safe Ramp Rs + Saturation Profiler L(I)
 *
 * ALGORITHM OVERVIEW
 * ──────────────────
 * 1. ALIGN (150 ms)
 *    Safe linear voltage ramp (30 V/s) locks the rotor along the d-axis at I1.
 *    theta_elec = 0 throughout. 0% overshoot across all motor sizes.
 *
 * 2. MEASURE_RS — Dual-filter 2-point steady-state DC measurement
 *    Vd1 = Rs * I1 + Vdead
 *    Vd2 = Rs * I2 + Vdead
 *    → Rs = (Vd2 − Vd1) / (I2 − I1)  [Dead-time completely cancels!]
 *    → Vdead = max(Vd1 − Rs * I1, 0)
 *
 * 3. FREQ_DETECT (10 ms)
 *    Quick 10-cycle probe at 1000 Hz to measure Z_1000.
 *    Automatically selects optimal frequency:
 *      - 4800 Hz: Whoop / Gimbal (Rs > 1.0 Ω or I_max <= 3A)
 *      - 250 Hz: Large Inductance Hub / Spindle (Z_1000 > 1.5 Ω)
 *      - 2400 Hz: FPV Drone / Slotless BLDC
 *
 * 4. MEASURE_SAT_PROFILE — 6-Level DC Bias + AC Injection + Exact ZOH Inversion
 *    Measures L at 6 DC bias currents (20%, 35%, 50%, 65%, 80%, 95% I_max).
 *    Direct feedforward bias Vd_bias = Vdead + Rs * I_bias (Current never crosses zero).
 *    Lock-in DFT demodulation with 1-sample Preload delay compensation.
 *    Extracts L using Exact Quadratic Discrete ZOH Inversion:
 *        (M − 1)·a^2 − 2·(M − cos(phi))·a + (M − 1) = 0, where a = exp(−Rs·Ts / L)
 *    Fits full saturation curve via Rational Least Squares:
 *        1 / L(I) = A + B · I^2 → L0 = 1 / A, Isat = sqrt(A / B), alpha = B / A.
 */

#include "motor_id.h"

#include <math.h>
#include <stdbool.h>

#include "flash_config.h"
#include "foc_config.h"
#include "foc_state_machine.h"

/*===========================================================================*/
/* Configuration Constants & Limits                                          */
/*===========================================================================*/

#define ID_MAX_DFT_N 192

/* Frequency generation tables and DFT buffers */
static float sin_table_cmd[ID_MAX_DFT_N];
static float cos_table_cmd[ID_MAX_DFT_N];
static float sin_table_demod[ID_MAX_DFT_N];
static float cos_table_demod[ID_MAX_DFT_N];
static float buf_s[ID_MAX_DFT_N];
static float buf_c[ID_MAX_DFT_N];
static float run_sum_s;
static float run_sum_c;
static uint16_t buf_idx;
static uint16_t step_in_cycle;
static uint16_t N_DFT;
static float sat_freq;
static float sat_omega;
static float sat_phase_inc;

/* Timekeeping */
static uint32_t id_timer_ms;
static uint32_t tick_counter;

/* Current low-pass filters for steady-state detection */
static float id_filt;
static float id_filt_fast;
static float id_filt_slow;

/* Align */
static float align_vd;

/* Measure Rs */
static uint8_t rs_sub;
static float rs_vd_out;
static float rs_id_target;
static float rs_vd_sum;
static float rs_id_sum;
static uint32_t rs_sum_count;
static float rs_vd1;
static float rs_id1;
static uint16_t rs_settle_counter;

/* Measure Saturation Profile */
static const float sat_current_levels[6] = {0.08f, 0.18f, 0.30f, 0.42f, 0.55f, 0.70f};
static uint8_t sat_level_idx;
static float sat_vd_bias;
static uint8_t sat_sub;
static uint16_t sat_settle_cnt;
static uint32_t meas_cycles;
static float v_inj;

static float sat_lut_i[6];
static float sat_lut_l[6];
static uint8_t sat_lut_count;

static float meas_accum_L[500];
static uint16_t meas_accum_count;

/* Public Result */
MotorID_Result_t id_result;

/*===========================================================================*/
/* Private Helper Functions                                                  */
/*===========================================================================*/

static void configure_freq(float f_probe) {
    sat_freq = f_probe;
    sat_omega = TWO_PI * sat_freq;
    sat_phase_inc = TWO_PI * sat_freq * CONTROL_PERIOD_F;
    N_DFT = (uint16_t)((float)PWM_FREQUENCY / sat_freq + 0.5f);
    if (N_DFT > ID_MAX_DFT_N) N_DFT = ID_MAX_DFT_N;
    if (N_DFT < 4) N_DFT = 4;

    /* Compensate exact 1.5-cycle PWM hardware delay (0.5 sample ADC + 1.0 sample Timer shadow
     * reload) */
    float delay_phase = 1.5f * sat_phase_inc;
    for (uint16_t k = 0; k < N_DFT; k++) {
        float phase_cmd = sat_phase_inc * (float)k;
        sin_table_cmd[k] = sinf(phase_cmd);
        cos_table_cmd[k] = cosf(phase_cmd);
        sin_table_demod[k] = sinf(phase_cmd - delay_phase);
        cos_table_demod[k] = cosf(phase_cmd - delay_phase);
        buf_s[k] = 0.0f;
        buf_c[k] = 0.0f;
    }
    run_sum_s = 0.0f;
    run_sum_c = 0.0f;
    buf_idx = 0;
    step_in_cycle = 0;
}

static float quick_median(float* arr, int n) {
    if (n <= 0) return 0.0f;
    for (int i = 1; i < n; i++) {
        float key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }
    if (n % 2 != 0) {
        return arr[n / 2];
    } else {
        return 0.5f * (arr[(n - 1) / 2] + arr[n / 2]);
    }
}

/*===========================================================================*/
/* Public API Functions                                                      */
/*===========================================================================*/

void MotorID_Init(void) {
    id_result.measured_rs = 0.0f;
    id_result.measured_ls = 0.0f;
    id_result.sat_isat = 0.0f;
    id_result.sat_alpha = 0.0f;
    id_result.measured_vdead = 0.0f;
    id_result.identified_deadtime_ns = 0.0f;
    id_result.selected_freq_hz = 2400.0f;
    id_result.state = MOTOR_ID_STATE_IDLE;
    id_result.error_code = 0;

    id_timer_ms = 0;
    tick_counter = 0;

    id_filt = 0.0f;
    id_filt_fast = 0.0f;
    id_filt_slow = 0.0f;

    align_vd = 0.0f;
    rs_sub = 0;
    sat_sub = 0;
    sat_level_idx = 0;
    sat_lut_count = 0;
    meas_accum_count = 0;

    configure_freq(2400.0f);
}

void MotorID_Start(void) {
    MotorID_Init();
    id_result.state = MOTOR_ID_STATE_ALIGN;
}

void MotorID_Stop(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
}

void MotorID_GetResults(MotorID_Result_t* results) {
    if (results != NULL) {
        *results = id_result;
    }
}

/*===========================================================================*/
/* Core State Machine Step (Called at 48 kHz PWM Interrupt)                  */
/*===========================================================================*/

void MotorID_RunStep(float id, float iq, float vbus, float* vd, float* vq) {
    (void)iq;
    *vq = 0.0f;
    float vd_out = 0.0f;

    if (id_result.state == MOTOR_ID_STATE_IDLE || id_result.state == MOTOR_ID_STATE_COMPLETE ||
        id_result.state == MOTOR_ID_STATE_ERROR) {
        *vd = 0.0f;
        return;
    }

    tick_counter++;
    if (tick_counter >= (PWM_FREQUENCY / 1000U)) {
        id_timer_ms++;
        tick_counter = 0;
    }

    /* Update current tracking filters */
    id_filt += 0.02f * (id - id_filt);
    id_filt_fast += 0.02f * (id - id_filt_fast);
    id_filt_slow += 0.002f * (id - id_filt_slow);

    float motor_max_curr = FlashConfig_Get()->motor_max_curr;
    if (motor_max_curr < 0.5f) motor_max_curr = 2.0f;
    float v_ramp_step = 30.0f * CONTROL_PERIOD_F; /* ~0.625 mV per PWM tick */

    /* ===================================================================== */
    /* 1. STATE_ALIGN (Safe 30 V/s Voltage Ramp to Align Rotor at I1)        */
    /* ===================================================================== */
    if (id_result.state == MOTOR_ID_STATE_ALIGN) {
        float target = 0.20f * motor_max_curr;
        if (target < 0.08f) target = 0.08f;
        float v_max = 0.35f * vbus;

        if (id_filt < target && align_vd < v_max) {
            align_vd += v_ramp_step;
        }
        vd_out = align_vd;

        if (id_timer_ms >= 150) {
            rs_sub = 0; /* SETTLE_I1 */
            rs_vd_out = align_vd;
            rs_vd_sum = 0.0f;
            rs_id_sum = 0.0f;
            rs_sum_count = 0;
            rs_settle_counter = 0;
            id_timer_ms = 0;
            id_result.state = MOTOR_ID_STATE_MEASURE_RS;
        }
    }

    /* ===================================================================== */
    /* 2. STATE_MEASURE_RS (2-Point DC: Settle I1 -> Ramp to I2 -> Settle I2)*/
    /* ===================================================================== */
    else if (id_result.state == MOTOR_ID_STATE_MEASURE_RS) {
        float v_max = 0.45f * vbus;
        float diff = fabsf(id_filt_fast - id_filt_slow);
        float is_flat_thr = g_foc.noise_profile.is_flat_thr + 0.010f * id_filt;
        bool is_flat = (diff < is_flat_thr);

        if (rs_sub == 0) { /* SETTLE_I1 */
            vd_out = rs_vd_out;
            if (is_flat && id_timer_ms > 15) {
                rs_settle_counter++;
                rs_vd_sum += rs_vd_out;
                rs_id_sum += id;
                rs_sum_count++;

                if (rs_settle_counter >= 360) { /* ~7.5 ms average */
                    rs_vd1 = rs_vd_sum / (float)rs_sum_count;
                    rs_id1 = rs_id_sum / (float)rs_sum_count;

                    rs_id_target = 0.50f * motor_max_curr;
                    if (rs_id_target < 2.5f * rs_id1) rs_id_target = 2.5f * rs_id1;

                    rs_sub = 1; /* RAMP_TO_I2 */
                    rs_settle_counter = 0;
                    rs_vd_sum = 0.0f;
                    rs_id_sum = 0.0f;
                    rs_sum_count = 0;
                    id_timer_ms = 0;
                }
            }
        } else if (rs_sub == 1) { /* RAMP_TO_I2 */
            if (id_filt < rs_id_target && rs_vd_out < v_max) {
                rs_vd_out += v_ramp_step;
            } else {
                rs_sub = 2; /* Transition to SETTLE_I2 */
                rs_settle_counter = 0;
                rs_vd_sum = 0.0f;
                rs_id_sum = 0.0f;
                rs_sum_count = 0;
                id_timer_ms = 0;
            }
            vd_out = rs_vd_out;
        } else if (rs_sub == 2) { /* SETTLE_I2 */
            vd_out = rs_vd_out;
            if (is_flat && id_timer_ms > 15) {
                rs_settle_counter++;
                rs_vd_sum += rs_vd_out;
                rs_id_sum += id;
                rs_sum_count++;

                if (rs_settle_counter >= 360) { /* ~7.5 ms average */
                    float vd2 = rs_vd_sum / (float)rs_sum_count;
                    float id2 = rs_id_sum / (float)rs_sum_count;

                    float delta_v = vd2 - rs_vd1;
                    float delta_i = id2 - rs_id1;

                    if (delta_i > 0.02f) {
                        float rs = delta_v / delta_i;
                        if (rs > 0.002f && rs < 50.0f) {
                            id_result.measured_rs = rs;
                            float vdead = rs_vd1 - rs * rs_id1;
                            if (vdead < 0.0f) vdead = 0.0f;
                            id_result.measured_vdead = vdead;
                            id_result.identified_deadtime_ns =
                                (vdead / vbus) * (1.0e9f / (float)PWM_FREQUENCY);

                            /* Enter Quick FREQ_DETECT Probe at 1000 Hz (10 ms) */
                            configure_freq(1000.0f);
                            rs_sub = 3; /* FREQ_DETECT_PROBE */
                            meas_cycles = 0;
                            buf_idx = 0;
                            id_timer_ms = 0;
                        } else {
                            id_result.error_code = 1;
                            id_result.state = MOTOR_ID_STATE_ERROR;
                        }
                    } else {
                        id_result.error_code = 7;
                        id_result.state = MOTOR_ID_STATE_ERROR;
                    }
                }
            }
        } else if (rs_sub == 3) { /* FREQ_DETECT_PROBE at 1000 Hz (10 ms) */
            float v_probe = 0.50f;
            float sin_cmd = sin_table_cmd[step_in_cycle];
            vd_out = id_result.measured_vdead + id_result.measured_rs * 0.15f * motor_max_curr +
                     v_probe * sin_cmd;

            buf_s[buf_idx] = id * sin_table_demod[step_in_cycle];
            buf_c[buf_idx] = id * cos_table_demod[step_in_cycle];

            buf_idx = (buf_idx + 1) % N_DFT;
            step_in_cycle = (step_in_cycle + 1) % N_DFT;
            meas_cycles++;

            if (meas_cycles >= (uint32_t)(N_DFT * 10)) { /* 10 cycles = 10 ms */
                float sum_s = 0.0f, sum_c = 0.0f;
                for (uint16_t k = 0; k < N_DFT; k++) {
                    sum_s += buf_s[k];
                    sum_c += buf_c[k];
                }
                float i_r = 2.0f * (sum_s / (float)N_DFT);
                float i_i = 2.0f * (sum_c / (float)N_DFT);
                float i_amp = sqrtf(i_r * i_r + i_i * i_i);
                float z_1000 = (i_amp > 1e-4f) ? (v_probe / i_amp) : 100.0f;
                float rs_sq = id_result.measured_rs * id_result.measured_rs;
                float x_sq = (z_1000 * z_1000 > rs_sq) ? (z_1000 * z_1000 - rs_sq) : 0.0f;
                float l_est_probe = sqrtf(x_sq) / (TWO_PI * 1000.0f);

                float f_chosen = 2400.0f;
                if (l_est_probe > 0.0004f || z_1000 > 3.0f) {
                    f_chosen = 250.0f; /* High Inductance > 400 uH -> 250 Hz */
                } else if (l_est_probe > 0.00008f || z_1000 > 1.0f) {
                    f_chosen = 1000.0f; /* Medium Inductance 80-400 uH -> 1000 Hz */
                } else if (l_est_probe > 0.000025f) {
                    f_chosen = 2400.0f; /* Low Inductance 25-80 uH -> 2400 Hz */
                } else {
                    f_chosen = 4800.0f; /* Ultra Low Inductance < 25 uH -> 4800 Hz */
                }

                configure_freq(f_chosen);
                id_result.selected_freq_hz = f_chosen;

                /* Advance to Standstill Saturation Profiler */
                sat_level_idx = 0;
                sat_lut_count = 0;
                sat_sub = 0;
                sat_settle_cnt = 0;
                id_timer_ms = 0;
                id_result.state = MOTOR_ID_STATE_MEASURE_SAT_PROFILE;
            }
        }
    }

    /* ===================================================================== */
    /* 3. STATE_MEASURE_SAT_PROFILE (Direct Bias + AC Injection + ZOH Inv)   */
    /* ===================================================================== */
    else if (id_result.state == MOTOR_ID_STATE_MEASURE_SAT_PROFILE) {
        float target_i_bias = sat_current_levels[sat_level_idx] * motor_max_curr;
        float v_max = 0.45f * vbus;

        sat_vd_bias = id_result.measured_vdead + id_result.measured_rs * target_i_bias;
        if (sat_vd_bias > v_max) sat_vd_bias = v_max;

        float est_l_guess = (sat_lut_count > 0) ? sat_lut_l[sat_lut_count - 1]
                                                : ((sat_freq <= 500.0f) ? 0.0020f : 20.0e-6f);
        float zhf = sqrtf(id_result.measured_rs * id_result.measured_rs +
                          (sat_omega * est_l_guess) * (sat_omega * est_l_guess));
        float target_ac_curr = clampf(0.12f * motor_max_curr, g_foc.noise_profile.i_inj_min, 0.50f);
        v_inj = clampf(target_ac_curr * zhf, 0.20f, 0.30f * vbus);

        float sin_cmd = sin_table_cmd[step_in_cycle];

        if (sat_sub == 0) { /* SETTLE_AT_DC_BIAS */
            vd_out = sat_vd_bias;
            sat_settle_cnt++;
            uint16_t settle_target = (sat_freq == 250.0f) ? 960 : 360; /* ~7.5 - 20 ms */
            if (sat_settle_cnt >= settle_target) {
                sat_sub = 1;
                meas_accum_count = 0;
                meas_cycles = 0;
                buf_idx = 0;
                run_sum_s = 0.0f;
                run_sum_c = 0.0f;
                for (uint16_t k = 0; k < N_DFT; k++) {
                    buf_s[k] = 0.0f;
                    buf_c[k] = 0.0f;
                }
                id_timer_ms = 0;
            }
        } else if (sat_sub == 1) { /* INJECT_SINE_ON_TOP_OF_DC */
            vd_out = sat_vd_bias + v_inj * sin_cmd;

            float new_s = id * sin_table_demod[step_in_cycle];
            float new_c = id * cos_table_demod[step_in_cycle];

            /* O(1) Sliding DFT: update running sum by adding new and subtracting oldest */
            run_sum_s += new_s - buf_s[buf_idx];
            run_sum_c += new_c - buf_c[buf_idx];

            buf_s[buf_idx] = new_s;
            buf_c[buf_idx] = new_c;

            buf_idx = (buf_idx + 1) % N_DFT;
            step_in_cycle = (step_in_cycle + 1) % N_DFT;
            meas_cycles++;

            /* Evaluate ZOH extraction once per AC period when step_in_cycle == 0 */
            if (step_in_cycle == 0 && meas_cycles > (uint32_t)(N_DFT * 2)) {
                float i_real = 2.0f * (run_sum_s / (float)N_DFT);
                float i_imag = 2.0f * (run_sum_c / (float)N_DFT);
                float i_mag_sq = i_real * i_real + i_imag * i_imag;

                if (i_mag_sq > 1e-10f) {
                    float Z_mag_sq = (v_inj * v_inj) / i_mag_sq;
                    float M = Z_mag_sq / (id_result.measured_rs * id_result.measured_rs);
                    float phi = sat_phase_inc;

                    /* Exact Quadratic Discrete ZOH Equation:
                     * (M - 1)·a^2 - 2·(M - cos(phi))·a + (M - 1) = 0 */
                    float A_quad = M - 1.0f;
                    float B_quad = -2.0f * (M - cosf(phi));
                    float C_quad = M - 1.0f;

                    float delta = B_quad * B_quad - 4.0f * A_quad * C_quad;
                    if (delta >= 0.0f && fabsf(A_quad) > 1e-7f) {
                        float sqrt_delta = sqrtf(delta);
                        float a1 = (-B_quad - sqrt_delta) / (2.0f * A_quad);
                        float a2 = (-B_quad + sqrt_delta) / (2.0f * A_quad);
                        float a_sol = (a1 > 0.0f && a1 < 1.0f) ? a1 : a2;
                        if (a_sol > 0.0f && a_sol < 0.999999f) {
                            float ls_sample =
                                -id_result.measured_rs * CONTROL_PERIOD_F / logf(a_sol);
                            if (ls_sample > 1e-7f && meas_accum_count < 500) {
                                meas_accum_L[meas_accum_count++] = ls_sample;
                            }
                        }
                    } else if (Z_mag_sq > id_result.measured_rs * id_result.measured_rs) {
                        float ls_sample =
                            sqrtf(Z_mag_sq - id_result.measured_rs * id_result.measured_rs) /
                            sat_omega;
                        if (ls_sample > 1e-7f && meas_accum_count < 500) {
                            meas_accum_L[meas_accum_count++] = ls_sample;
                        }
                    }
                }
            }

            uint32_t target_cycles = (uint32_t)N_DFT * ((sat_freq <= 500.0f) ? 12 : 25);
            if (meas_cycles >= target_cycles) {
                if (meas_accum_count >= 2) {
                    float l_est_level = quick_median(meas_accum_L, meas_accum_count);
                    sat_lut_i[sat_lut_count] = id_filt;
                    sat_lut_l[sat_lut_count] = l_est_level;
                    sat_lut_count++;
                }

                sat_level_idx++;
                if (sat_level_idx < 6) {
                    sat_sub = 0;
                    sat_settle_cnt = 0;
                    id_timer_ms = 0;
                } else {
                    /* Full Saturation Curve Collected! Exact Rational Inversion Fit: 1/L = A + B *
                     * I^2 */
                    if (sat_lut_count >= 3) {
                        float sum_x = 0.0f, sum_y = 0.0f;
                        float x[6], y[6];
                        for (uint8_t i = 0; i < sat_lut_count; i++) {
                            x[i] = sat_lut_i[i] * sat_lut_i[i];
                            y[i] = 1.0f / sat_lut_l[i];
                            sum_x += x[i];
                            sum_y += y[i];
                        }
                        float mean_x = sum_x / (float)sat_lut_count;
                        float mean_y = sum_y / (float)sat_lut_count;

                        float num = 0.0f, den = 0.0f;
                        for (uint8_t i = 0; i < sat_lut_count; i++) {
                            float dx = x[i] - mean_x;
                            num += dx * (y[i] - mean_y);
                            den += dx * dx;
                        }

                        float B = (den > 1e-12f) ? (num / den) : 0.0f;
                        float A = mean_y - B * mean_x;

                        id_result.measured_ls = (A > 0.0f) ? (1.0f / A) : sat_lut_l[0];
                        id_result.sat_alpha = (A > 0.0f && B > 0.0f) ? (B / A) : 0.0f;
                        id_result.sat_isat = (A > 0.0f && B > 0.0f) ? sqrtf(A / B) : 0.0f;

                        id_result.state = MOTOR_ID_STATE_COMPLETE;
                    } else {
                        id_result.error_code = 3;
                        id_result.state = MOTOR_ID_STATE_ERROR;
                    }
                }
            }
        }
    }

    *vd = vd_out;
}
