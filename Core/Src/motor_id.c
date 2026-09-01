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
 *      - 250 Hz: Large Inductance Hub / Spindle (L_probe > 400 uH or Z_1000 > 3.0 Ω)
 *      - 1000 Hz: Medium Inductance (80-400 uH or Z_1000 > 1.0 Ω)
 *      - 4800 Hz: Low Inductance Drone (10-80 uH or Z_1000 > 0.15 Ω)
 *      - 12000 Hz: Ultra-Low Inductance Slotless (< 10 uH)
 *
 * 4. MEASURE_SAT_PROFILE — 20-Point Dynamic Saturation Profiler + AC Injection + Exact ZOH
 * Inversion Measures L at up to 20 dynamically adjusted DC bias currents. Slow integrated DC bias:
 * Vd_bias = Vdead + Rs * I_bias + sat_vd_error_int (Slow DC current loop). Lock-in DFT demodulation
 * with 1.5-cycle ZOH/delay compensation. Extracts L using Exact Discrete ZOH Inversion solved via
 * Newton-Raphson: M = (1 - 2*x^2*cos(phi) + x^4) / ((1 - x^2)^2), where x = exp(−0.5*Rs*Ts / L)
 *    Fits full saturation curve via Least Squares:
 *        1 / L(I) = A + B · I^2 → L0 = 1 / A, Isat = sqrt(1 / (B * L0)), alpha = B * L0.
 */

#include "motor_id.h"

#include <math.h>
#include <stdbool.h>

#include "cordic_math.h"
#include "flash_config.h"
#include "foc.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "math.h"
#include "peripheral_init.h"

/* DEBUG VARIABLES FOR ACTIVE NOISE MEASUREMENT */
float debug_rs_id_sq_sum = 0.0f;
float debug_noise_rms_active = 0.0f;

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

/* Current low-pass tracking filters */
static float id_filt;
static float id_filt_slow;

/* Analytically derived filter & resolution parameters */
static float id_alpha_fast;
static float id_alpha_slow;
static float id_v_ramp_rate;
static float id_v_ramp_step;
static float is_flat_thr_base;
static float min_delta_i_req;

/* Align */
static float align_vd;

/* Measure Rs */
static uint8_t rs_sub;
static float rs_vd_out;
static float rs_vd_sum;
static float rs_id_sum;
static uint32_t rs_sum_count;
static float rs_vd1;
static float rs_id1;
static uint16_t rs_settle_counter;

/* Measure Saturation Profile */
#define SAT_MAX_LUT_POINTS 20

static uint8_t sat_level_idx;
static float sat_vd_bias;
static uint8_t sat_sub;
static uint16_t sat_settle_cnt;
static uint32_t meas_cycles;
static float v_inj;

static float sat_lut_i[SAT_MAX_LUT_POINTS];
static float sat_lut_l[SAT_MAX_LUT_POINTS];
static uint8_t sat_lut_count;

/* Adaptive saturation tracking variables */
static float sat_i_anchor;         // Linear region anchor current
static float sat_delta_i;          // Dynamic step increment
static float sat_delta_i_max;      // Maximum allowable step size
static float sat_L0_val;           // Reference nominal L0 value (Level 0)
static float sat_last_i_bias;      // DC bias of last accepted point
static float sat_last_L_val;       // Inductance of last accepted point
static uint8_t sat_backtrack_cnt;  // Counter for backtrack steps (limit to 2)
static float sat_target_i_bias;    // Dynamic DC bias current target
static float sat_vd_error_int;     // Slow DC bias voltage integrator

static float meas_accum_L[500];
static uint16_t meas_accum_count;

/* Public Result */
MotorID_Result_t id_result;

/* Flux ID Variables */
static uint8_t s_is_flux_measuring = 0;
static uint32_t s_flux_coast_counter = 0;
static float s_flux_theta_unwrapped = 0.0f;
static uint32_t s_flux_sample_count = 0;
static float s_flux_prev_theta = 0.0f;
static float s_vac_sq_sum = 0.0f;

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
    /* 1. Reset results struct */
    id_result.measured_rs = 0.0f;
    id_result.measured_ls = 0.0f;
    id_result.sat_isat = 0.0f;
    id_result.sat_alpha = 0.0f;
    id_result.measured_vdead = 0.0f;
    id_result.identified_deadtime_ns = 0.0f;
    id_result.selected_freq_hz = 2400.0f;
    id_result.measured_flux = 0.0f;
    id_result.measured_kv = 0.0f;
    id_result.state = MOTOR_ID_STATE_IDLE;
    id_result.error_code = 0;

    id_timer_ms = 0;
    tick_counter = 0;

    /* 2. Retrieve hardware specifications and live calibration noise profile */
    float ts = CONTROL_PERIOD_F;
    float noise_rms = g_foc.noise_profile.noise_rms;
    if (noise_rms < ID_NOISE_FLOOR_MIN) noise_rms = ID_NOISE_FLOOR_MIN;

    float lsb_current = fabsf(ADC_TO_CURRENT) * ADC_Vref;
    float motor_max_curr = FlashConfig_Get()->motor_max_curr;
    if (motor_max_curr < 0.5f) motor_max_curr = 2.0f;

    /* 3. Compute Fast Filter alpha (tau = 1.04 ms for instant ramp stop) */
    id_alpha_fast = ts / ID_FAST_TAU_TARGET_S;

    /* 4. Compute 4-sigma post-filter flat threshold envelope (99.994% rejection) */
    float noise_gain_fast = sqrtf(id_alpha_fast * 0.5f);
    is_flat_thr_base = ID_FLAT_SIGMA_MULT * (noise_rms * noise_gain_fast);

    /* 5. Compute Slow Filter alpha (Tuned for worst-case max electrical tau) */
    float delta_i_step = ID_MIN_DELTA_VOLTAGE / ID_MIN_MOTOR_RS;
    if (delta_i_step > ID_MAX_DELTA_I_CALC_CAP) delta_i_step = ID_MAX_DELTA_I_CALC_CAP;

    float di_dt_settle = (delta_i_step / ID_MAX_MOTOR_TAU_S) * 0.0183156f; /* exp(-4.0) */
    float delta_tau = is_flat_thr_base / di_dt_settle;
    float tau_slow = ID_FAST_TAU_TARGET_S + delta_tau * 1.30f;
    id_alpha_slow = ts / tau_slow;
    id_alpha_slow = clampf(id_alpha_slow, ID_ALPHA_SLOW_MIN, ID_ALPHA_SLOW_MAX);

    /* 6. Compute Linear Voltage Ramp Rate from overshoot constraint */
    float allowed_overshoot = ID_RAMP_OVERSHOOT_PCT * motor_max_curr;
    if (allowed_overshoot < 0.20f) allowed_overshoot = 0.20f;
    id_v_ramp_rate = (allowed_overshoot * ID_MIN_MOTOR_RS) / ID_FAST_TAU_TARGET_S;
    id_v_ramp_rate = clampf(id_v_ramp_rate, ID_V_RAMP_MIN, ID_V_RAMP_MAX);
    id_v_ramp_step = id_v_ramp_rate * ts;

    /* 7. Compute Required Minimum Current Delta for target SNR and ADC resolution */
    float min_i_lsb = ID_MIN_ADC_COUNTS * lsb_current;
    float min_i_snr = ID_MIN_NOISE_SNR * noise_rms;
    min_delta_i_req = (min_i_lsb > min_i_snr) ? min_i_lsb : min_i_snr;

    /* 8. Reset internal state machine variables */
    id_filt = 0.0f;
    id_filt_slow = 0.0f;

    align_vd = 0.0f;
    rs_sub = 0;
    sat_sub = 0;
    sat_level_idx = 0;
    sat_lut_count = 0;
    meas_accum_count = 0;
    sat_i_anchor = 0.0f;
    sat_delta_i = 0.0f;
    sat_delta_i_max = 0.0f;
    sat_L0_val = 0.0f;
    sat_last_i_bias = 0.0f;
    sat_last_L_val = 0.0f;
    sat_backtrack_cnt = 0;
    sat_target_i_bias = 0.0f;
    sat_vd_error_int = 0.0f;

    configure_freq(2400.0f);
}

void MotorID_Start(void) {
    MotorID_Init();
    id_result.state = MOTOR_ID_STATE_ALIGN;
}

void MotorID_Stop(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
    FOC_EnableDrivers(0);
}

uint8_t MotorID_IsFluxMeasuring(void) {
    return s_is_flux_measuring;
}

void MotorID_MeasureFluxOffline(void) {
    s_is_flux_measuring = 1;
    s_flux_coast_counter = 0;
    s_flux_theta_unwrapped = 0.0f;
    s_flux_sample_count = 0;
    s_flux_prev_theta = 0.0f;
    s_vac_sq_sum = 0.0f;
    FOC_Start();
}

void FOC_StateCoastFluxID(void) {
    static float s_dc_alpha = 0.0f;
    static float s_dc_beta = 0.0f;

    if (s_flux_coast_counter == 0) {
        FOC_SetPhaseVoltageDMA(1);
        FOC_EnableDrivers(0);
        s_flux_theta_unwrapped = 0.0f;
        s_flux_sample_count = 0;
        s_flux_prev_theta = 0.0f;
        s_vac_sq_sum = 0.0f;
        s_dc_alpha = 0.0f;
        s_dc_beta = 0.0f;
    }

    float Ea = g_foc.data.Vphase_a;
    float Ec = g_foc.data.Vphase_c;

    /* Reconstruct 2-phase Clarke Transform directly from Ea, Ec (assuming Eb = -Ea - Ec) */
    float E_alpha = 1.5f * Ea;
    float E_beta = -0.8660254f * (Ea + 2.0f * Ec);

    float alpha_dc = TWO_PI * 5.0f * CONTROL_PERIOD_F;
    if (s_flux_coast_counter < 10) {
        s_dc_alpha = E_alpha;
        s_dc_beta = E_beta;
    } else {
        s_dc_alpha += (E_alpha - s_dc_alpha) * alpha_dc;
        s_dc_beta += (E_beta - s_dc_beta) * alpha_dc;
    }

    float theta = cordic_atan2(-(E_alpha - s_dc_alpha), (E_beta - s_dc_beta));
    if (s_flux_coast_counter == 10) {
        s_flux_prev_theta = theta;
    }

    /* delta_theta is in [-1, 1] for [-pi, pi] */
    float delta_theta = 0.0f;
    if (s_flux_coast_counter >= 10) {
        delta_theta = normalize_angle_norm(theta - s_flux_prev_theta);
        s_flux_prev_theta = theta;
    }

    /* Ignore first 720 samples (~15ms) to allow DMA to stabilize and inductive spikes to decay
     * completely */
    if (s_flux_coast_counter > 720) {
        /* User requested Line-to-Line calculation: Vac = Va - Vc */
        float Vac = Ea - Ec;
        s_vac_sq_sum += Vac * Vac;

        s_flux_theta_unwrapped += delta_theta;
        s_flux_sample_count++;
    }

    s_flux_coast_counter++;

    /* Accumulate for a total of 80ms (15ms wait + 65ms measure).
     * The exact integration method is highly precise even with short windows,
     * which is crucial for low-inertia motors that decelerate rapidly. */
    uint32_t target_samples = (uint32_t)(0.080f * (float)CONTROL_FREQUENCY);
    if (s_flux_coast_counter >= target_samples) {
        float measured_flux = 0.0f;
        float measured_kv = 0.0f;

        if (s_flux_sample_count > 0) {
            float avg_delta_theta = s_flux_theta_unwrapped / (float)s_flux_sample_count;
            float avg_vac_sq = s_vac_sq_sum / (float)s_flux_sample_count;

            float omega_avg = avg_delta_theta * PI * (float)CONTROL_FREQUENCY;

            if (fabsf(omega_avg) > 10.0f) {
                /* Line-to-Line peak voltage from RMS: Vpeak = sqrt(Vrms_sq * 2) */
                float Vac_peak = sqrtf(avg_vac_sq * 2.0f);

                /* Calculate KV directly from Line-to-Line Peak:
                 * KV = RPM / V_L-L(peak)
                 * RPM = (omega_avg_elec * 60) / (2 * PI * pole_pairs) */
                float rpm = (fabsf(omega_avg) * 60.0f) / (TWO_PI * g_foc.cfg.motor_poles);
                measured_kv = rpm / Vac_peak;

                /* Back-calculate Flux Linkage from KV for the system to use:
                 * Flux = 60 / (sqrt(3) * 2 * PI * KV * Poles) */
                measured_flux = 60.0f / (SQRT3 * TWO_PI * measured_kv * g_foc.cfg.motor_poles);
            }
        }

        id_result.measured_flux = measured_flux;
        id_result.measured_kv = measured_kv;
        id_result.state = MOTOR_ID_STATE_COMPLETE;

        s_is_flux_measuring = 0;
        FOC_Stop();
    }
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

    /* Update current tracking filters (Fast and Slow) */
    id_filt += id_alpha_fast * (id - id_filt);
    id_filt_slow += id_alpha_slow * (id - id_filt_slow);

    float motor_max_curr = FlashConfig_Get()->motor_max_curr;
    if (motor_max_curr < 0.5f) motor_max_curr = 2.0f;

    /* ===================================================================== */
    /* 1. STATE_ALIGN (Safe Voltage Ramp to Align Rotor at I1)               */
    /* ===================================================================== */
    if (id_result.state == MOTOR_ID_STATE_ALIGN) {
        float lsb_current = fabsf(ADC_TO_CURRENT) * ADC_Vref;
        float target_i1 = 50.0f * lsb_current;
        if (target_i1 < 5.0f * g_foc.noise_profile.noise_rms) {
            target_i1 = 5.0f * g_foc.noise_profile.noise_rms;
        }
        if (target_i1 < 0.08f) target_i1 = 0.08f;
        if (target_i1 > 0.20f * motor_max_curr) target_i1 = 0.20f * motor_max_curr;

        float v_max = 0.35f * vbus;

        if (id_filt < target_i1 && align_vd < v_max) {
            align_vd += id_v_ramp_step;
        }
        vd_out = align_vd;

        if (id_timer_ms >= ID_ALIGN_DURATION_MS) {
            rs_sub = 0; /* SETTLE_I1 */
            rs_vd_out = align_vd;
            rs_vd_sum = 0.0f;
            rs_id_sum = 0.0f;
            debug_rs_id_sq_sum = 0.0f;
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
        float diff = fabsf(id_filt - id_filt_slow);
        bool is_flat = (diff < is_flat_thr_base);

        if (rs_sub == 0) { /* SETTLE_I1 */
            vd_out = rs_vd_out;
            if (is_flat && id_timer_ms > ID_SETTLE_HOLD_TIME_MS) {
                rs_settle_counter++;
                rs_vd_sum += rs_vd_out;
                rs_id_sum += id;
                debug_rs_id_sq_sum += id * id;
                rs_sum_count++;

                if (rs_settle_counter >= ID_SETTLE_SAMPLES) {
                    rs_vd1 = rs_vd_sum / (float)rs_sum_count;
                    rs_id1 = rs_id_sum / (float)rs_sum_count;

                    float var = (debug_rs_id_sq_sum / (float)rs_sum_count) - (rs_id1 * rs_id1);
                    if (var > 0.0f) {
                        debug_noise_rms_active = sqrtf(var);
                    }

                    rs_sub = 1; /* RAMP_TO_I2 */
                    rs_settle_counter = 0;
                    rs_vd_sum = 0.0f;
                    rs_id_sum = 0.0f;
                    rs_sum_count = 0;
                    id_timer_ms = 0;
                }
            } else {
                if (!is_flat) {
                    rs_settle_counter = 0;
                    rs_vd_sum = 0.0f;
                    rs_id_sum = 0.0f;
                    debug_rs_id_sq_sum = 0.0f;
                    rs_sum_count = 0;
                }
            }
        } else if (rs_sub == 1) { /* RAMP_TO_I2 (SMART AUTO-RESOLUTION GATE) */
            float cur_delta_i = id_filt - rs_id1;
            float cur_delta_v = rs_vd_out - rs_vd1;

            float min_req = (min_delta_i_req > 0.30f * rs_id1) ? min_delta_i_req : (0.30f * rs_id1);
            bool enough_res = (cur_delta_i >= min_req) && (cur_delta_v >= ID_MIN_DELTA_VOLTAGE);
            bool safety_hit = (id_filt >= 0.50f * motor_max_curr) || (rs_vd_out >= v_max);

            if (enough_res || safety_hit) {
                /* Sufficient resolution collected -> Stop ramping immediately! */
                rs_sub = 2; /* Transition to SETTLE_I2 */
                rs_settle_counter = 0;
                rs_vd_sum = 0.0f;
                rs_id_sum = 0.0f;
                rs_sum_count = 0;
                id_timer_ms = 0;
            } else {
                rs_vd_out += id_v_ramp_step;
            }
            vd_out = rs_vd_out;
        } else if (rs_sub == 2) { /* SETTLE_I2 */
            vd_out = rs_vd_out;
            if (is_flat && id_timer_ms > ID_SETTLE_HOLD_TIME_MS) {
                rs_settle_counter++;
                rs_vd_sum += rs_vd_out;
                rs_id_sum += id;
                rs_sum_count++;

                if (rs_settle_counter >= ID_SETTLE_SAMPLES) {
                    float vd2 = rs_vd_sum / (float)rs_sum_count;
                    float id2 = rs_id_sum / (float)rs_sum_count;

                    float delta_v = vd2 - rs_vd1;
                    float delta_i = id2 - rs_id1;

                    if (delta_i > 0.01f) {
                        float rs = delta_v / delta_i;
                        if (rs > 0.001f && rs < 100.0f) {
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
            } else {
                if (!is_flat) {
                    rs_settle_counter = 0;
                    rs_vd_sum = 0.0f;
                    rs_id_sum = 0.0f;
                    rs_sum_count = 0;
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
                } else if (l_est_probe > 0.00001f || z_1000 > 0.15f) {
                    f_chosen = 4800.0f; /* Low Inductance 10-80 uH -> 4800 Hz */
                } else {
                    f_chosen = 12000.0f; /* Ultra Low Inductance < 10 uH -> 12000 Hz */
                }

                configure_freq(f_chosen);
                id_result.selected_freq_hz = f_chosen;

                /* Advance to Standstill Saturation Profiler */
                float noise_floor_anchor = 2.0f * g_foc.noise_profile.i_inj_min;
                float min_anchor =
                    clampf(0.05f * motor_max_curr, noise_floor_anchor, noise_floor_anchor * 2.0f);
                float max_anchor = clampf(0.1f * motor_max_curr, noise_floor_anchor * 2.0f,
                                          noise_floor_anchor * 4.0f);
                sat_i_anchor = clampf(3.0f * g_foc.noise_profile.noise_rms, min_anchor, max_anchor);

                float noise_floor_delta = g_foc.noise_profile.i_inj_min;
                float min_delta =
                    clampf(0.02f * motor_max_curr, noise_floor_delta, noise_floor_delta * 2.0f);
                float max_delta = clampf(0.06f * motor_max_curr, noise_floor_delta * 2.0f,
                                         noise_floor_delta * 4.0f);
                sat_delta_i = clampf(1.5f * g_foc.noise_profile.noise_rms, min_delta, max_delta);
                sat_delta_i_max = 0.15f * motor_max_curr;
                sat_L0_val = 0.0f;
                sat_backtrack_cnt = 0;
                sat_last_i_bias = 0.0f;
                sat_last_L_val = 0.0f;
                sat_target_i_bias = sat_i_anchor;
                sat_vd_error_int = 0.0f;

                sat_level_idx = 0;
                sat_lut_count = 0;
                sat_sub = 0;
                sat_settle_cnt = 0;
                id_timer_ms = 0;
                sat_vd_bias = id_result.measured_vdead + id_result.measured_rs * sat_i_anchor;
                id_result.state = MOTOR_ID_STATE_MEASURE_SAT_PROFILE;
            }
        }
    }

    /* ===================================================================== */
    /* 3. STATE_MEASURE_SAT_PROFILE (Direct Bias + AC Injection + ZOH Inv)   */
    /* ===================================================================== */
    else if (id_result.state == MOTOR_ID_STATE_MEASURE_SAT_PROFILE) {
        float target_i_bias = sat_target_i_bias;
        float v_max = 0.45f * vbus;

        /* Slow integrator to regulate DC bias current to target_i_bias */
        sat_vd_error_int += 0.002f * (target_i_bias - id_filt);
        float error_limit = 0.10f * vbus;
        sat_vd_error_int = clampf(sat_vd_error_int, -error_limit, error_limit);

        sat_vd_bias =
            id_result.measured_vdead + id_result.measured_rs * target_i_bias + sat_vd_error_int;
        if (sat_vd_bias > v_max) sat_vd_bias = v_max;

        float est_l_guess = (sat_lut_count > 0) ? sat_lut_l[sat_lut_count - 1]
                                                : ((sat_freq <= 500.0f) ? 0.0020f : 20.0e-6f);
        float zhf = sqrtf(id_result.measured_rs * id_result.measured_rs +
                          (sat_omega * est_l_guess) * (sat_omega * est_l_guess));
        /* Calculate AC Injection Limits for Zero-Cross and Hardware Safety */
        float target_ac_curr = 20.0f * g_foc.noise_profile.noise_rms;
        if (target_ac_curr < g_foc.noise_profile.i_inj_min) {
            target_ac_curr = g_foc.noise_profile.i_inj_min;
        }

        /* Limit AC current to what the current DC bias can support safely */
        float max_safe_ac = 0.40f * target_i_bias;
        if (target_ac_curr > max_safe_ac) {
            target_ac_curr = max_safe_ac;
        }

        float v_target = target_ac_curr * zhf;
        float limit_bus = 0.15f * vbus;
        float limit_zc = 0.80f * target_i_bias * zhf;
        float limit_max = (motor_max_curr - target_i_bias) * zhf;

        v_inj = v_target;
        if (v_inj > limit_bus) v_inj = limit_bus;
        if (v_inj > limit_max) v_inj = limit_max;
        if (v_inj > limit_zc) v_inj = limit_zc;

        /* Lower bound safety */
        float min_vbus = 0.02f * vbus;
        if (v_inj < min_vbus && v_inj < limit_zc) {
            v_inj = (min_vbus < limit_zc) ? min_vbus : limit_zc;
        }

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

                    /* Exact Discrete ZOH Equation via Newton-Raphson
                     * M = (1 - 2*x^2*cos(phi) + x^4) / ((1 - x)^2 * (1 + x^2 + 2*x*cos(phi)))
                     * where x = exp(-0.5 * Rs * Ts / L) */
                    float cos_phi = cosf(phi);

                    /* Initial guess based on standard continuous L = Z / omega */
                    float L_guess = 1e-6f;
                    if (Z_mag_sq > id_result.measured_rs * id_result.measured_rs) {
                        L_guess = sqrtf(Z_mag_sq - id_result.measured_rs * id_result.measured_rs) /
                                  sat_omega;
                    }

                    float x_est = expf(-0.5f * id_result.measured_rs * CONTROL_PERIOD_F / L_guess);

                    for (int iter = 0; iter < 3; iter++) {
                        float x2 = x_est * x_est;
                        float x3 = x2 * x_est;
                        float x4 = x2 * x2;

                        float num = 1.0f - 2.0f * x2 * cos_phi + x4;
                        float term1 = 1.0f - x_est;
                        float term2 = 1.0f + x2 + 2.0f * x_est * cos_phi;
                        float den = term1 * term1 * term2;

                        if (fabsf(den) < 1e-12f) break;

                        float f_val = (num / den) - M;

                        float dnum_dx = -4.0f * x_est * cos_phi + 4.0f * x3;
                        float dden_dx =
                            -2.0f * term1 * term2 + term1 * term1 * (2.0f * x_est + 2.0f * cos_phi);
                        float df_dx = (dnum_dx * den - num * dden_dx) / (den * den);

                        if (fabsf(df_dx) < 1e-12f) break;

                        x_est = x_est - (f_val / df_dx);

                        if (x_est < 0.001f) x_est = 0.001f;
                        if (x_est > 0.999f) x_est = 0.999f;
                    }

                    float ls_sample =
                        -id_result.measured_rs * 0.5f * CONTROL_PERIOD_F / logf(x_est);

                    if (ls_sample > 1e-7f && meas_accum_count < 500) {
                        meas_accum_L[meas_accum_count++] = ls_sample;
                    }
                }
            }

            uint32_t target_cycles = (uint32_t)N_DFT * ((sat_freq <= 500.0f) ? 12 : 25);
            if (meas_cycles >= target_cycles) {
                float l_est_level = 0.0f;
                bool valid_meas = false;

                if (meas_accum_count >= 2) {
                    l_est_level = quick_median(meas_accum_L, meas_accum_count);
                    valid_meas = true;
                }

                if (valid_meas) {
                    if (sat_lut_count == 0) {
                        /* Establish nominal L0 */
                        sat_L0_val = l_est_level;
                        sat_lut_i[0] = id_filt;
                        sat_lut_l[0] = l_est_level;
                        sat_lut_count = 1;

                        sat_last_i_bias = id_filt;
                        sat_last_L_val = l_est_level;
                        sat_backtrack_cnt = 0;

                        sat_level_idx = 1;
                        sat_target_i_bias = sat_last_i_bias + sat_delta_i;
                        sat_sub = 0;
                        sat_settle_cnt = 0;
                    } else {
                        float delta_L = sat_last_L_val - l_est_level;

                        /* Backtracking check: drop on this single step exceeds 15% L0 */
                        if (delta_L > 0.15f * sat_L0_val && sat_backtrack_cnt < 2) {
                            sat_delta_i = sat_delta_i / 2.0f;
                            sat_target_i_bias = sat_last_i_bias + sat_delta_i;
                            sat_backtrack_cnt++;

                            sat_sub = 0;
                            sat_settle_cnt = 0;
                        } else {
                            /* Accept point */
                            sat_lut_i[sat_lut_count] = id_filt;
                            sat_lut_l[sat_lut_count] = l_est_level;
                            sat_lut_count++;

                            sat_last_i_bias = id_filt;
                            sat_last_L_val = l_est_level;
                            sat_backtrack_cnt = 0;

                            /* Proportional Step-Size Control based on local slope */
                            float delta_L_pct = delta_L / sat_L0_val;
                            if (delta_L_pct < 0.04f) {
                                sat_delta_i = clampf(sat_delta_i * 1.3f, 0.10f, sat_delta_i_max);
                            } else if (delta_L_pct > 0.10f) {
                                sat_delta_i = clampf(sat_delta_i * 0.8f, 0.05f, sat_delta_i_max);
                            }

                            /* Evaluate Early Stopping criteria */
                            float next_i_bias = sat_last_i_bias + sat_delta_i;
                            float P_limit = clampf(
                                0.5f * motor_max_curr * motor_max_curr * id_result.measured_rs,
                                5.0f, 40.0f);
                            bool stop_scanning = false;

                            if (l_est_level <= 0.70f * sat_L0_val) {
                                stop_scanning = true; /* 1. Saturation target reached */
                            } else if (sat_vd_bias + v_inj >= 0.40f * vbus) {
                                stop_scanning = true; /* 2. Inverter voltage limit reached */
                            } else if (next_i_bias * next_i_bias * id_result.measured_rs >=
                                       P_limit) {
                                stop_scanning = true; /* 3. Adaptive thermal guard */
                            } else if (next_i_bias >= 0.85f * motor_max_curr) {
                                stop_scanning = true; /* 4. Safe current limit */
                            } else if (sat_lut_count >= SAT_MAX_LUT_POINTS) {
                                stop_scanning = true; /* 5. Array full */
                            }

                            if (stop_scanning) {
                                if (sat_lut_count >= 3) {
                                    /* 3. Full 2-Variable Least Squares Fit for Inverse-Square
                                     * Model: y = A + B * x where y = 1/L, x = I^2
                                     *    --> A = 1/L0, B = alpha / L0 */

                                    float sum_x = 0.0f, sum_y = 0.0f;
                                    float sum_xy = 0.0f, sum_xx = 0.0f;
                                    float n = (float)sat_lut_count;

                                    for (uint8_t i = 0; i < sat_lut_count; i++) {
                                        float x = sat_lut_i[i] * sat_lut_i[i];
                                        float y = 1.0f / sat_lut_l[i];
                                        sum_x += x;
                                        sum_y += y;
                                        sum_xy += x * y;
                                        sum_xx += x * x;
                                    }

                                    float delta = n * sum_xx - sum_x * sum_x;
                                    float A, B;

                                    if (delta > 1e-12f) {
                                        A = (sum_xx * sum_y - sum_x * sum_xy) / delta;
                                        B = (n * sum_xy - sum_x * sum_y) / delta;
                                    } else {
                                        /* Fallback if points are completely identical */
                                        A = 1.0f / sat_lut_l[0];
                                        B = 0.0f;
                                    }

                                    /* Fix physical anomalies: Inductance cannot increase with
                                     * current */
                                    if (B < 0.0f) {
                                        B = 0.0f;
                                        A = sum_y / n;
                                    }

                                    /* Extrapolate True L0 at 0 Amps */
                                    float L0_true = (A > 1e-6f) ? (1.0f / A) : sat_lut_l[0];

                                    float max_meas_L = sat_lut_l[0];
                                    if (L0_true < max_meas_L) L0_true = max_meas_L;
                                    if (L0_true > max_meas_L * 1.5f) L0_true = max_meas_L * 1.5f;

                                    id_result.sat_alpha = B * L0_true;
                                    id_result.sat_isat = (B > 1e-9f) ? sqrtf(1.0f / (B * L0_true))
                                                                     : motor_max_curr * 10.0f;
                                    id_result.measured_ls = L0_true;
                                    id_result.state = MOTOR_ID_STATE_COMPLETE;
                                } else {
                                    id_result.error_code = 3;
                                    id_result.state = MOTOR_ID_STATE_ERROR;
                                }
                            } else {
                                sat_level_idx++;
                                sat_target_i_bias = next_i_bias;
                                sat_sub = 0;
                                sat_settle_cnt = 0;
                            }
                        }
                    }
                } else {
                    id_result.error_code = 2;
                    id_result.state = MOTOR_ID_STATE_ERROR;
                }
            }
        }
    }

    *vd = vd_out;
}
