/**
 * @file motor_id.c
 * @brief Motor Parameter Identification - Safe Ramp Rs + Saturation Profiler L(I)
 *
 * ALGORITHM OVERVIEW
 * ------------------
 * 1. ALIGN (150 ms)
 *    Safe linear voltage ramp (30 V/s) locks the rotor along the d-axis at I1.
 *    theta_elec = 0 throughout. 0% overshoot across all motor sizes.
 *
 * 2. MEASURE_RS - 2-point DC voltage injection (Settle-Ramp) with dual IIR filtering
 *    Vd1 = Rs * I1 + Vdead
 *    Vd2 = Rs * I2 + Vdead
 *    -> Rs = (Vd2 - Vd1) / (I2 - I1)  [Dead-time completely cancels!]
 *    -> Vdead = max(Vd1 - Rs * I1, 0)
 *
 * 3. FREQ_DETECT (10 ms)
 *    Quick 10-cycle probe at 1000 Hz to measure Z_1000.
 *    Automatically selects optimal frequency:
 *      - 250 Hz: Large Inductance Hub / Spindle (L_probe > 400 uH or Z_1000 > 3.0 Ohm)
 *      - 1000 Hz: Medium Inductance (80-400 uH or Z_1000 > 1.0 Ohm)
 *      - 4800 Hz: Low Inductance Drone (10-80 uH or Z_1000 > 0.15 Ohm)
 *      - 12000 Hz: Ultra-Low Inductance Slotless (< 10 uH)
 *
 * 4. MEASURE_SAT_PROFILE - 20-Point Dynamic Saturation Profiler + AC Injection + Exact ZOH
 * Inversion Measures L at up to 20 dynamically adjusted DC bias currents. Slow integrated DC bias:
 * Vd_bias = Vdead + Rs * I_bias + sat_vd_error_int (Slow DC current loop). Lock-in DFT demodulation
 * with 1.5-cycle ZOH/delay compensation. Extracts L using Exact Discrete ZOH Inversion solved via
 * Newton-Raphson: M = (1 - 2*x^2*cos(phi) + x^4) / ((1 - x^2)^2), where x = exp(-0.5*Rs*Ts / L)
 *    Fits full saturation curve via Least Squares:
 *        1 / L(I) = A + B * I^2 -> L0 = 1 / A, Isat = sqrt(1 / (B * L0)), alpha = B * L0.
 */

#include "motor_id.h"

#include <math.h>
#include <stdbool.h>

#include "cordic_math.h"
#include "foc.h"
#include "foc_config.h"
#include "foc_state_machine.h"

/*===========================================================================*/
/* Configuration Constants & Limits                                          */
/*===========================================================================*/

#define ID_MAX_DFT_N 192

/* DFT buffers and frequency generation variables */
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
static float sat_phase_inc_norm;
static float sat_delay_phase_norm;
static float sat_cos_half_phi;

/* Timekeeping */
static uint32_t id_timer_ms;

/* Current low-pass tracking filters */
static volatile float id_filt;
static float id_filt_slow;

/* Analytically derived filter & resolution parameters */
static float id_alpha_fast;
static float id_alpha_slow;
static float id_v_ramp_step;
static float is_flat_thr_base;
static float min_delta_i_req;

/* Align */
static float align_vd;
static float align_target_i;
static float id_motor_max_curr;

/* Measure Rs */
volatile static uint8_t rs_sub;
static float rs_vd_out;
static volatile float rs_vd_sum;
static volatile float rs_id_sum;
static volatile float rs_probe_accum_s;
static volatile float rs_probe_accum_c;
static volatile uint32_t rs_sum_count;
static float rs_vd1;
static float rs_id1;

/* Measure Saturation Profile */
#define SAT_MAX_LUT_POINTS 20

static float sat_vd_bias;
volatile static uint8_t sat_sub;
static uint16_t sat_settle_cnt;
volatile static uint32_t meas_cycles;
static float v_inj;

static float sat_lut_i[SAT_MAX_LUT_POINTS];
static float sat_lut_l[SAT_MAX_LUT_POINTS];
static uint8_t sat_lut_count;

/* Adaptive saturation tracking variables */
static float sat_i_anchor;        /* Linear region anchor current */
static float sat_delta_i;         /* Dynamic step increment */
static float sat_delta_i_max;     /* Maximum allowable step size */
static float sat_L0_val;          /* Reference nominal L0 value (Level 0) */
static float sat_last_i_bias;     /* DC bias of last accepted point */
static float sat_last_L_val;      /* Inductance of last accepted point */
static uint8_t sat_backtrack_cnt; /* Counter for backtrack steps (limit to 2) */
static float sat_target_i_bias;   /* Dynamic DC bias current target */
static float sat_vd_error_int;    /* Slow DC bias voltage integrator */

#define SAT_MAX_ACCUM_SAMPLES 32
static float meas_accum_i_sq[SAT_MAX_ACCUM_SAMPLES];
volatile static uint16_t meas_accum_count;

/* Shared variables between FastTask (48kHz) and SlowTask (1kHz) */
static volatile uint8_t s_rs_accum_en = 0;
static volatile uint8_t s_rs_accum_done = 0;
static float rs_probe_vd_bias = 0.0f;
static float rs_probe_v_inj = 0.0f;

/* Public Result */
MotorID_Result_t id_result;

static uint32_t s_flux_coast_counter = 0;
static float s_cycle_vac_sum = 0.0f;
static float s_cycle_vac_sq_sum = 0.0f;
static uint32_t s_cycle_sample_count = 0;
static uint8_t s_zc_state = 0;
static uint32_t s_zc_cycle_count = 0;
static uint32_t s_last_zc_sample = 0;
static uint32_t s_valid_cycle_count = 0;
static float s_accum_flux = 0.0f;
static float s_accum_kv = 0.0f;
static uint8_t s_spin_detected = 0;
static int8_t s_flux_direction = 1;
static volatile uint8_t s_inertia_pending = 0;
static float s_saved_accel = 0.0f;

/*===========================================================================*/
/* Private Helper Functions                                                  */
/*===========================================================================*/

static void configure_freq(float f_probe) {
    sat_freq = f_probe;
    sat_omega = TWO_PI * sat_freq;
    sat_phase_inc = TWO_PI * sat_freq * g_foc.dt;
    sat_phase_inc_norm = 2.0f * sat_freq * g_foc.dt;
    sat_delay_phase_norm = 2.0f * sat_phase_inc_norm;
    sat_cos_half_phi = 1.0f;
    N_DFT = (uint16_t)(g_foc.cfg.pwm_frequency / sat_freq + 0.5f);
    if (N_DFT > ID_MAX_DFT_N) N_DFT = ID_MAX_DFT_N;
    if (N_DFT < 4) N_DFT = 4;

    /* Peak Update discrete phase lag:
     * - 2.0 dt: Total discrete delay from command calculation at Peak (k)
     *   to active execution in period [k+1, k+2] and ADC sampling at Peak (k+2).
     * - Pure ZOH: Output voltage is constant across the entire PWM period,
     *   no half-step mixing attenuation (sat_cos_half_phi = 1.0f). */
    for (uint16_t k = 0; k < N_DFT; k++) {
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

static float calculate_zoh_ls(float i_mag_sq) {
    if (i_mag_sq <= 1e-10f) return 0.0f;

    float v_inj_eff = v_inj * sat_cos_half_phi;
    float Z_mag_sq = (v_inj_eff * v_inj_eff) / i_mag_sq;
    float rs_sq = id_result.measured_rs * id_result.measured_rs;
    float M = Z_mag_sq / rs_sq;
    float phi = sat_phase_inc;
    float cos_phi = cosf(phi);
    float ls_sample = 0.0f;

    /* Exact Discrete ZOH Analytical Formula */
    if (M > 1.0001f) {
        float p = (M - cos_phi) / (M - 1.0f);
        if (p > 1.0f) {
            float disc = p * p - 1.0f;
            float a_est = p - sqrtf(disc);
            if (a_est > 1e-7f && a_est < 0.999999f) {
                ls_sample = -id_result.measured_rs * g_foc.dt / logf(a_est);
            }
        }
    }
    if (ls_sample <= 1e-7f) {
        float x_react = (Z_mag_sq > rs_sq) ? sqrtf(Z_mag_sq - rs_sq) : 0.0f;
        ls_sample = x_react / sat_omega;
    }
    return ls_sample;
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
    id_result.measured_flux = 0.0f;
    id_result.measured_kv = 0.0f;
    id_result.measured_b0 = 0.0f;
    id_result.measured_inertia = 0.0f;
    id_result.detected_direction = 0;
    id_result.state = MOTOR_ID_STATE_IDLE;
    id_result.error_code = 0;

    id_timer_ms = 0;

    /* 2. Retrieve hardware specifications and live calibration noise profile */
    float ts = g_foc.dt;
    float noise_rms = g_foc.cfg.cal_noise_rms;
    if (noise_rms < ID_NOISE_FLOOR_MIN) noise_rms = ID_NOISE_FLOOR_MIN;

    float lsb_current = fabsf(ADC_TO_CURRENT) * ADC_Vref;
    float motor_max_curr = FlashConfig_Get()->motor_max_curr;
    if (motor_max_curr < 0.5f) motor_max_curr = 2.0f;
    id_motor_max_curr = motor_max_curr;

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
    float id_v_ramp_rate = (allowed_overshoot * ID_MIN_MOTOR_RS) / ID_FAST_TAU_TARGET_S;
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
    float target_i1 = 50.0f * lsb_current;
    if (target_i1 < 5.0f * noise_rms) {
        target_i1 = 5.0f * noise_rms;
    }
    if (target_i1 < 0.08f) target_i1 = 0.08f;
    if (target_i1 > 0.20f * motor_max_curr) target_i1 = 0.20f * motor_max_curr;
    align_target_i = target_i1;

    rs_sub = 0;
    sat_sub = 0;
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

    s_rs_accum_en = 0;
    s_rs_accum_done = 0;
    rs_probe_vd_bias = 0.0f;
    rs_probe_v_inj = 0.0f;

    configure_freq(2400.0f);
}

void MotorID_Start(void) {
    if (fabsf(g_foc.cfg.pwm_frequency - 48000.0f) > 1.0f) {
        id_result.state = MOTOR_ID_STATE_ERROR;
        id_result.error_code = MOTOR_ID_ERR_INVALID_PWM_FREQ;
        return;
    }
    MotorID_Init();
    id_result.state = MOTOR_ID_STATE_ALIGN;
}

void MotorID_Stop(void) {
    if (id_result.state != MOTOR_ID_STATE_COMPLETE && id_result.state != MOTOR_ID_STATE_ERROR) {
        id_result.state = MOTOR_ID_STATE_IDLE;
    }
    if (s_inertia_pending) {
        s_inertia_pending = 0;
        if (s_saved_accel > 0.0f) {
            g_foc.cfg.speed_ramp_accel = s_saved_accel;
        }
    }
    FOC_EnableDrivers(0);
}

void MotorID_MeasureFluxOffline(void) {
    if (g_foc.status.state != FOC_STATE_IDLE) {
        return;
    }
    if (fabsf(g_foc.cfg.pwm_frequency - 48000.0f) > 1.0f) {
        id_result.state = MOTOR_ID_STATE_ERROR;
        id_result.error_code = MOTOR_ID_ERR_INVALID_PWM_FREQ;
        return;
    }
    id_result.state = MOTOR_ID_STATE_MEASURE_FLUX;
    id_result.error_code = 0;
    s_flux_coast_counter = 0;
    s_cycle_vac_sum = 0.0f;
    s_cycle_vac_sq_sum = 0.0f;
    s_cycle_sample_count = 0;
    s_zc_state = 0;
    s_zc_cycle_count = 0;
    s_last_zc_sample = 0;
    s_valid_cycle_count = 0;
    s_accum_flux = 0.0f;
    s_accum_kv = 0.0f;
    s_spin_detected = 0;
    s_flux_direction = 1;

    FOC_EnableDrivers(0);
    g_foc.status.state = FOC_STATE_COAST_FLUX_ID;
}

void FOC_StateCoastFluxID(void) {
    if (s_flux_coast_counter == 0) {
        FOC_EnableDrivers(0);
    }

    float Ea = g_foc.data.Vphase_a;
    float Ec = g_foc.data.Vphase_c;
    /* Differential Line-to-Line terminal voltage Vac = Ea - Ec.
     * Cancels out all floating neutral point shift Vn(t), diode clipping,
     * and 3rd harmonics 100%, leaving a pure, clean, zero-centered sinusoid Vac. */
    float Vac = Ea - Ec;

    /* Accumulate RMS statistics for current cycle (DC-rejecting variance method) */
    s_cycle_vac_sum += Vac;
    s_cycle_vac_sq_sum += Vac * Vac;
    s_cycle_sample_count++;

    /* Detect user motion */
    if (fabsf(Vac) > ID_FLUX_MIN_VAC) {
        s_spin_detected = 1;
    }

    /* Zero-Crossing Schmitt Trigger:
     * Positive rising threshold: +ID_FLUX_MIN_VAC
     * Negative re-arm threshold:  -ID_FLUX_MIN_VAC
     * Eliminates ADC noise ripple around zero (~0.02V) completely. */
    if (s_zc_state == 0) {
        if (Vac > ID_FLUX_MIN_VAC) {
            s_zc_state = 1;
            s_zc_cycle_count++;
            uint32_t now = s_flux_coast_counter;

            if (s_zc_cycle_count > 1) {
                uint32_t cycle_samples = now - s_last_zc_sample;
                /* Valid electrical frequency range: 5 Hz to 1000 Hz
                 * At 48kHz: 48 samples (1000 Hz) to 9600 samples (5 Hz) */
                if (cycle_samples >= 48 && cycle_samples <= 9600) {
                    float n_samp = (float)s_cycle_sample_count;
                    float mean_vac = s_cycle_vac_sum / n_samp;
                    float variance = (s_cycle_vac_sq_sum / n_samp) - (mean_vac * mean_vac);
                    if (variance < 0.0f) variance = 0.0f;
                    float Vac_peak = SQRT2 * sqrtf(variance);
                    if (Vac_peak >= ID_FLUX_MIN_VAC) {
                        float f_elec = (float)g_foc.cfg.pwm_frequency / (float)cycle_samples;
                        float omega_elec = TWO_PI * f_elec;
                        float rpm = (f_elec * 60.0f) / g_foc.cfg.motor_poles;

                        float cycle_flux = Vac_peak / (SQRT3 * omega_elec);
                        float cycle_kv = rpm / Vac_peak;

                        /* Direction detection: when Ea is crossing rising, if Ec < 0 then Phase A
                         * leads (Forward) */
                        if (s_valid_cycle_count == 0) {
                            s_flux_direction = (Ec < 0.0f) ? 1 : -1;
                        }

                        s_accum_flux += cycle_flux;
                        s_accum_kv += cycle_kv;
                        s_valid_cycle_count++;

                        /* Real-time telemetry feedback */
                        g_foc.data.omega_elec = omega_elec;
                    }
                }
            }

            s_last_zc_sample = now;
            s_cycle_vac_sum = 0.0f;
            s_cycle_vac_sq_sum = 0.0f;
            s_cycle_sample_count = 0;
        }
    } else {
        if (Vac < -ID_FLUX_MIN_VAC) {
            s_zc_state = 0;
        }
    }

    s_flux_coast_counter++;

    /* Condition 1: Target valid cycles reached (6 cycles) */
    if (s_valid_cycle_count >= 6) {
        id_result.measured_flux = s_accum_flux / (float)s_valid_cycle_count;
        id_result.measured_kv = s_accum_kv / (float)s_valid_cycle_count;
        id_result.detected_direction = s_flux_direction;
        id_result.state = MOTOR_ID_STATE_COMPLETE;

        FOC_Stop();
        return;
    }

    /* Condition 2: Motor stopped spinning after moving (no ZC for > 0.3s = 14400 samples) */
    if (s_spin_detected && (s_flux_coast_counter - s_last_zc_sample > 14400)) {
        if (s_valid_cycle_count >= 3) {
            id_result.measured_flux = s_accum_flux / (float)s_valid_cycle_count;
            id_result.measured_kv = s_accum_kv / (float)s_valid_cycle_count;
            id_result.detected_direction = s_flux_direction;
            id_result.state = MOTOR_ID_STATE_COMPLETE;

            FOC_Stop();
            return;
        } else {
            /* Insufficient cycles before stopping: re-arm and wait for user to flick again */
            s_spin_detected = 0;
            s_zc_cycle_count = 0;
            s_cycle_vac_sum = 0.0f;
            s_cycle_vac_sq_sum = 0.0f;
            s_cycle_sample_count = 0;
        }
    }

    /* Condition 3: Total timeout (10.0 seconds) */
    if (s_flux_coast_counter >= (uint32_t)(10.0f * g_foc.cfg.pwm_frequency)) {
        if (s_valid_cycle_count >= 3) {
            id_result.measured_flux = s_accum_flux / (float)s_valid_cycle_count;
            id_result.measured_kv = s_accum_kv / (float)s_valid_cycle_count;
            id_result.detected_direction = s_flux_direction;
            id_result.state = MOTOR_ID_STATE_COMPLETE;
        } else {
            id_result.state = MOTOR_ID_STATE_ERROR;
            id_result.error_code = 7;
        }

        FOC_Stop();
        return;
    }
}

void MotorID_GetResults(MotorID_Result_t* results) {
    if (results != NULL) {
        *results = id_result;
    }
}

/*===========================================================================*/
/* 48 kHz Fast ISR Task (Zero-Wait State CCMRAM, Execution < 0.4 us)          */
/*===========================================================================*/

void MotorID_FastTask(float id, float* vd, float* vq) {
    *vq = 0.0f;

    if (id_result.state == MOTOR_ID_STATE_IDLE || id_result.state == MOTOR_ID_STATE_COMPLETE ||
        id_result.state == MOTOR_ID_STATE_ERROR) {
        *vd = 0.0f;
        return;
    }

    id_filt += id_alpha_fast * (id - id_filt);
    id_filt_slow += id_alpha_slow * (id - id_filt_slow);

    if (id_result.state == MOTOR_ID_STATE_ALIGN) {
        float v_max = 0.35f * g_foc.data.Vbus;
        if (id_filt < align_target_i && align_vd < v_max) {
            align_vd += id_v_ramp_step;
        }
        *vd = align_vd;
    } else if (id_result.state == MOTOR_ID_STATE_MEASURE_RS) {
        if (rs_sub == 0 || rs_sub == 2) {
            /* Settle states I1 and I2 */
            if (s_rs_accum_en) {
                float diff = fabsf(id_filt - id_filt_slow);
                if (diff < is_flat_thr_base) {
                    rs_vd_sum += rs_vd_out;
                    rs_id_sum += id;
                    rs_sum_count++;
                    if (rs_sum_count >= ID_SETTLE_SAMPLES) {
                        s_rs_accum_en = 0;
                        s_rs_accum_done = 1;
                    }
                } else {
                    /* Reset accumulation if not flat to ensure consecutive clean samples */
                    rs_vd_sum = 0.0f;
                    rs_id_sum = 0.0f;
                    rs_sum_count = 0;
                }
            }
            *vd = rs_vd_out;
        } else if (rs_sub == 1) {
            /* Smooth voltage ramp for RAMP_TO_I2 */
            float v_max = 0.45f * g_foc.data.Vbus;
            float cur_delta_i = id_filt - rs_id1;
            float cur_delta_v = rs_vd_out - rs_vd1;

            float min_req = (min_delta_i_req > 0.30f * rs_id1) ? min_delta_i_req : (0.30f * rs_id1);
            bool enough_res = (cur_delta_i >= min_req) && (cur_delta_v >= ID_MIN_DELTA_VOLTAGE);
            bool safety_hit = (id_filt >= 0.50f * id_motor_max_curr) || (rs_vd_out >= v_max);

            if (enough_res || safety_hit) {
                /* Sufficient resolution collected -> Stop ramping immediately! */
                rs_sub = 2; /* Transition to SETTLE_I2 */
                s_rs_accum_en = 0;
                s_rs_accum_done = 0;
                rs_vd_sum = 0.0f;
                rs_id_sum = 0.0f;
                rs_sum_count = 0;
            } else {
                rs_vd_out += id_v_ramp_step;
            }
            *vd = rs_vd_out;
        } else if (rs_sub == 3) {
            /* FREQ_DETECT_PROBE at 1000 Hz */
            float phase_cmd_norm = sat_phase_inc_norm * (float)step_in_cycle;
            float dummy_cos, sin_cmd;
            cordic_sincos(normalize_angle_norm(phase_cmd_norm), &dummy_cos, &sin_cmd);
            *vd = rs_probe_vd_bias + rs_probe_v_inj * sin_cmd;

            if (meas_cycles >= (uint32_t)(N_DFT * 5) && meas_cycles < (uint32_t)(N_DFT * 10)) {
                float phase_demod_norm = phase_cmd_norm - sat_delay_phase_norm;
                float cos_demod, sin_demod;
                cordic_sincos(normalize_angle_norm(phase_demod_norm), &cos_demod, &sin_demod);
                rs_probe_accum_s += id * sin_demod;
                rs_probe_accum_c += id * cos_demod;
            }
            step_in_cycle = (step_in_cycle + 1) % N_DFT;
            meas_cycles++;
        } else {
            /* rs_sub >= 4: Mute AC injection during frequency reconfiguration */
            *vd = rs_probe_vd_bias;
        }
    } else if (id_result.state == MOTOR_ID_STATE_MEASURE_SAT_PROFILE) {
        if (sat_sub == 0) {
            /* Anti-windup non-negative clamp prevents wiping out Rs*target_i_bias below deadtime */
            sat_vd_error_int += 0.002f * (sat_target_i_bias - id_filt);
            sat_vd_error_int = clampf(sat_vd_error_int, 0.0f, 0.10f * g_foc.data.Vbus);

            sat_vd_bias = id_result.measured_vdead + id_result.measured_rs * sat_target_i_bias +
                          sat_vd_error_int;
            if (sat_vd_bias < id_result.measured_vdead) sat_vd_bias = id_result.measured_vdead;
            float v_max = 0.45f * g_foc.data.Vbus;
            if (sat_vd_bias > v_max) sat_vd_bias = v_max;

            *vd = sat_vd_bias;
        } else if (sat_sub == 1) {
            /* INJECT_SINE_ON_TOP_OF_DC: DC integrator frozen to prevent DFT drift */
            float phase_cmd_norm = sat_phase_inc_norm * (float)step_in_cycle;
            float dummy_cos, sin_cmd;
            cordic_sincos(normalize_angle_norm(phase_cmd_norm), &dummy_cos, &sin_cmd);
            *vd = sat_vd_bias + v_inj * sin_cmd;

            float phase_demod_norm = phase_cmd_norm - sat_delay_phase_norm;
            float cos_demod, sin_demod;
            cordic_sincos(normalize_angle_norm(phase_demod_norm), &cos_demod, &sin_demod);

            float new_s = id * sin_demod;
            float new_c = id * cos_demod;

            run_sum_s += new_s - buf_s[buf_idx];
            run_sum_c += new_c - buf_c[buf_idx];

            buf_s[buf_idx] = new_s;
            buf_c[buf_idx] = new_c;

            buf_idx = (buf_idx + 1) % N_DFT;
            step_in_cycle = (step_in_cycle + 1) % N_DFT;
            meas_cycles++;

            /* Synchronous extraction once per AC period at zero-crossing (step_in_cycle == 0) */
            if (step_in_cycle == 0 && meas_cycles > (uint32_t)(N_DFT * 2)) {
                float i_real = 2.0f * (run_sum_s / (float)N_DFT);
                float i_imag = 2.0f * (run_sum_c / (float)N_DFT);
                float i_mag_sq = i_real * i_real + i_imag * i_imag;

                if (i_mag_sq > 1e-10f && meas_accum_count < SAT_MAX_ACCUM_SAMPLES) {
                    meas_accum_i_sq[meas_accum_count++] = i_mag_sq;
                }
            }
        }
    }
}

/*===========================================================================*/
/* 1 kHz Slow Task (Called from FOC_SlowTask in Flash)                       */
/*===========================================================================*/

void MotorID_SlowTask(void) {
    if (id_result.state == MOTOR_ID_STATE_IDLE || id_result.state == MOTOR_ID_STATE_COMPLETE ||
        id_result.state == MOTOR_ID_STATE_ERROR) {
        return;
    }

    id_timer_ms++;

    float motor_max_curr = FlashConfig_Get()->motor_max_curr;
    if (motor_max_curr < 0.5f) motor_max_curr = 2.0f;
    float vbus = g_foc.data.Vbus;

    /* ===================================================================== */
    /* 1. STATE_ALIGN (Safe Voltage Ramp to Align Rotor at I1)               */
    /* ===================================================================== */
    if (id_result.state == MOTOR_ID_STATE_ALIGN) {
        if (id_timer_ms >= ID_ALIGN_DURATION_MS) {
            rs_sub = 0; /* SETTLE_I1 */
            rs_vd_out = align_vd;
            rs_vd_sum = 0.0f;
            rs_id_sum = 0.0f;
            rs_sum_count = 0;
            s_rs_accum_en = 0;
            s_rs_accum_done = 0;
            id_timer_ms = 0;
            id_result.state = MOTOR_ID_STATE_MEASURE_RS;
        }
    }

    /* ===================================================================== */
    /* 2. STATE_MEASURE_RS (2-Point DC: Settle I1 -> Ramp to I2 -> Settle I2)*/
    /* ===================================================================== */
    else if (id_result.state == MOTOR_ID_STATE_MEASURE_RS) {
        if (rs_sub == 0) { /* SETTLE_I1 */
            if (!s_rs_accum_en && !s_rs_accum_done) {
                if (id_timer_ms > ID_SETTLE_HOLD_TIME_MS) {
                    rs_vd_sum = 0.0f;
                    rs_id_sum = 0.0f;
                    rs_sum_count = 0;
                    s_rs_accum_en = 1;
                }
            } else if (s_rs_accum_done) {
                s_rs_accum_done = 0;
                s_rs_accum_en = 0;
                rs_vd1 = rs_vd_sum / (float)rs_sum_count;
                rs_id1 = rs_id_sum / (float)rs_sum_count;

                rs_sub = 1; /* RAMP_TO_I2 */
                rs_vd_sum = 0.0f;
                rs_id_sum = 0.0f;
                rs_sum_count = 0;
                id_timer_ms = 0;
            }

            /* Safety timeout for Settle I1 (500 ms) */
            if (id_timer_ms > 500) {
                id_result.error_code = 1;
                id_result.state = MOTOR_ID_STATE_ERROR;
            }
        } else if (rs_sub == 1) { /* RAMP_TO_I2 */
            if (id_timer_ms > 1000) {
                id_result.error_code = 1;
                id_result.state = MOTOR_ID_STATE_ERROR;
            }
        } else if (rs_sub == 2) { /* SETTLE_I2 */
            if (!s_rs_accum_en && !s_rs_accum_done) {
                if (id_timer_ms > ID_SETTLE_HOLD_TIME_MS) {
                    rs_vd_sum = 0.0f;
                    rs_id_sum = 0.0f;
                    rs_sum_count = 0;
                    s_rs_accum_en = 1;
                }
            } else if (s_rs_accum_done) {
                s_rs_accum_done = 0;
                s_rs_accum_en = 0;
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
                            (vdead / vbus) * (1.0e9f / g_foc.cfg.pwm_frequency);
                        g_foc.cfg.deadtime_ns = id_result.identified_deadtime_ns;
                        rs_sub = 4; /* Mute AC injection during configuration */
                        configure_freq(1000.0f);
                        rs_probe_vd_bias = id_result.measured_vdead +
                                           id_result.measured_rs * 0.15f * motor_max_curr;
                        float v_max_bias = 0.40f * vbus;
                        if (rs_probe_vd_bias > v_max_bias) rs_probe_vd_bias = v_max_bias;
                        rs_probe_v_inj = 0.50f;
                        rs_probe_accum_s = 0.0f;
                        rs_probe_accum_c = 0.0f;
                        meas_cycles = 0;
                        buf_idx = 0;
                        step_in_cycle = 0;
                        id_timer_ms = 0;
                        rs_sub = 3; /* FREQ_DETECT_PROBE */
                    } else {
                        id_result.error_code = 1;
                        id_result.state = MOTOR_ID_STATE_ERROR;
                    }
                } else {
                    id_result.error_code = 7;
                    id_result.state = MOTOR_ID_STATE_ERROR;
                }
            }

            /* Safety timeout for Settle I2 (500 ms) */
            if (id_timer_ms > 500) {
                id_result.error_code = 1;
                id_result.state = MOTOR_ID_STATE_ERROR;
            }
        } else if (rs_sub == 3) { /* FREQ_DETECT_PROBE at 1000 Hz (10 ms) */
            if (meas_cycles >= (uint32_t)(N_DFT * 10)) {
                float total_samples = 5.0f * (float)N_DFT;
                float i_r = 2.0f * (rs_probe_accum_s / total_samples);
                float i_i = 2.0f * (rs_probe_accum_c / total_samples);
                float i_amp = sqrtf(i_r * i_r + i_i * i_i);

                float v_probe_eff = rs_probe_v_inj;
                float z_1000 = (i_amp > 1e-4f) ? (v_probe_eff / i_amp) : 100.0f;
                float rs_sq = id_result.measured_rs * id_result.measured_rs;
                float x_sq = (z_1000 * z_1000 > rs_sq) ? (z_1000 * z_1000 - rs_sq) : 0.0f;
                float l_est_probe = sqrtf(x_sq) / (TWO_PI * 1000.0f);

                /* Safe frequency selection based on inductance L and electrical tau_e = L / Rs */
                float tau_e_est =
                    (id_result.measured_rs > 1e-4f) ? (l_est_probe / id_result.measured_rs) : 0.0f;
                float f_chosen = 2400.0f;
                if (l_est_probe > 0.0004f) {
                    f_chosen = 250.0f; /* High Inductance > 400 uH -> 250 Hz */
                } else if (l_est_probe > 0.00008f) {
                    f_chosen = 1000.0f; /* Medium Inductance 80-400 uH -> 1000 Hz */
                } else if (l_est_probe > 0.000020f && tau_e_est >= 0.00020f) {
                    f_chosen = 4800.0f; /* Low Inductance (tau_e >= 200 us) -> 4800 Hz */
                } else {
                    f_chosen =
                        12000.0f; /* Ultra Low Inductance (< 20 uH or tau_e < 200 us) -> 12000 Hz */
                }

                rs_sub = 4; /* Mute AC injection so FastTask won't access tables */
                configure_freq(f_chosen);
                id_result.selected_freq_hz = f_chosen;

                /* Advance to Standstill Saturation Profiler */
                float noise_floor_anchor = 2.0f * g_foc.cfg.cal_i_inj_min;
                sat_i_anchor =
                    clampf(0.05f * motor_max_curr, noise_floor_anchor, 0.20f * motor_max_curr);

                float noise_floor_delta = g_foc.cfg.cal_i_inj_min;
                float min_delta_factor = (f_chosen >= 10000.0f) ? 0.008f : 0.02f;
                sat_delta_i_max =
                    (f_chosen >= 10000.0f) ? (0.05f * motor_max_curr) : (0.15f * motor_max_curr);
                sat_delta_i =
                    clampf(min_delta_factor * motor_max_curr, noise_floor_delta, sat_delta_i_max);
                sat_L0_val = 0.0f;
                sat_backtrack_cnt = 0;
                sat_last_i_bias = 0.0f;
                sat_last_L_val = 0.0f;
                sat_target_i_bias = sat_i_anchor;
                sat_vd_error_int = 0.0f;

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

        float est_l_guess = (sat_lut_count > 0) ? sat_lut_l[sat_lut_count - 1]
                                                : ((sat_freq <= 500.0f) ? 0.0020f : 20.0e-6f);
        float zhf = sqrtf(id_result.measured_rs * id_result.measured_rs +
                          (sat_omega * est_l_guess) * (sat_omega * est_l_guess));

        /* Calculate AC Injection Limits for Zero-Cross and Hardware Safety */
        float target_ac_curr = 20.0f * g_foc.cfg.cal_noise_rms;
        if (target_ac_curr < g_foc.cfg.cal_i_inj_min) {
            target_ac_curr = g_foc.cfg.cal_i_inj_min;
        }

        float max_safe_ac = 0.40f * target_i_bias;
        if (target_ac_curr > max_safe_ac) {
            target_ac_curr = max_safe_ac;
        }

        float v_target = target_ac_curr * zhf;
        float limit_bus = 0.15f * vbus;
        float limit_zc = 0.80f * target_i_bias * zhf;
        float limit_max = (motor_max_curr - target_i_bias) * zhf;

        float v_safe = v_target;
        if (v_safe > limit_bus) v_safe = limit_bus;
        if (v_safe > limit_max) v_safe = limit_max;
        if (v_safe > limit_zc) v_safe = limit_zc;

        float min_vbus = 0.02f * vbus;
        if (v_safe < min_vbus && v_safe < limit_zc) {
            v_safe = (min_vbus < limit_zc) ? min_vbus : limit_zc;
        }
        v_inj = v_safe;

        if (sat_sub == 0) { /* SETTLE_AT_DC_BIAS */
            sat_settle_cnt++;
            uint16_t settle_target_ms = (sat_freq <= 500.0f) ? 25 : 15; /* ~15 - 25 ms */
            if (sat_settle_cnt >= settle_target_ms) {
                meas_accum_count = 0;
                meas_cycles = 0;
                buf_idx = 0;
                step_in_cycle = 0;
                run_sum_s = 0.0f;
                run_sum_c = 0.0f;
                for (uint16_t k = 0; k < N_DFT; k++) {
                    buf_s[k] = 0.0f;
                    buf_c[k] = 0.0f;
                }
                id_timer_ms = 0;
                sat_sub = 1;
            }
        } else if (sat_sub == 1) { /* INJECT_SINE_ON_TOP_OF_DC */
            uint32_t target_cycles = (uint32_t)N_DFT * ((sat_freq <= 500.0f) ? 12 : 25);
            if (meas_cycles >= target_cycles || id_timer_ms >= 120) {
                float l_est_level = 0.0f;
                bool valid_meas = false;

                __disable_irq();
                uint8_t count = (meas_accum_count > SAT_MAX_ACCUM_SAMPLES)
                                    ? SAT_MAX_ACCUM_SAMPLES
                                    : (uint8_t)meas_accum_count;
                float local_accum[SAT_MAX_ACCUM_SAMPLES];
                for (uint8_t i = 0; i < count; i++) {
                    local_accum[i] = meas_accum_i_sq[i];
                }
                __enable_irq();

                if (count >= 2) {
                    float i_mag_sq_median = quick_median(local_accum, count);
                    l_est_level = calculate_zoh_ls(i_mag_sq_median);
                    if (l_est_level > 1e-7f) {
                        valid_meas = true;
                    }
                }

                if (valid_meas) {
                    if (sat_lut_count == 0) {
                        sat_L0_val = l_est_level;
                        sat_lut_i[0] = id_filt;
                        sat_lut_l[0] = l_est_level;
                        sat_lut_count = 1;

                        sat_last_i_bias = id_filt;
                        sat_last_L_val = l_est_level;
                        sat_backtrack_cnt = 0;

                        sat_target_i_bias = sat_last_i_bias + sat_delta_i;
                        sat_sub = 0;
                        sat_settle_cnt = 0;
                        sat_vd_error_int = 0.0f;
                    } else {
                        float delta_L = sat_last_L_val - l_est_level;

                        if (delta_L > 0.15f * sat_L0_val && sat_backtrack_cnt < 2) {
                            sat_delta_i = sat_delta_i / 2.0f;
                            sat_target_i_bias = sat_last_i_bias + sat_delta_i;
                            sat_backtrack_cnt++;

                            sat_sub = 0;
                            sat_settle_cnt = 0;
                            sat_vd_error_int = 0.0f;
                        } else {
                            sat_lut_i[sat_lut_count] = id_filt;
                            sat_lut_l[sat_lut_count] = l_est_level;
                            sat_lut_count++;

                            sat_last_i_bias = id_filt;
                            sat_last_L_val = l_est_level;
                            sat_backtrack_cnt = 0;

                            float delta_L_pct = delta_L / sat_L0_val;
                            if (delta_L_pct < 0.04f) {
                                sat_delta_i = clampf(sat_delta_i * 1.3f, 0.10f, sat_delta_i_max);
                            } else if (delta_L_pct > 0.10f) {
                                sat_delta_i = clampf(sat_delta_i * 0.8f, 0.05f, sat_delta_i_max);
                            }

                            float next_i_bias = sat_last_i_bias + sat_delta_i;
                            float P_limit = clampf(
                                0.5f * motor_max_curr * motor_max_curr * id_result.measured_rs,
                                5.0f, 40.0f);
                            bool stop_scanning = false;

                            if (l_est_level <= 0.70f * sat_L0_val) {
                                stop_scanning = true;
                            } else if (sat_vd_bias + v_inj >= 0.40f * vbus) {
                                stop_scanning = true;
                            } else if (next_i_bias * next_i_bias * id_result.measured_rs >=
                                       P_limit) {
                                stop_scanning = true;
                            } else if (next_i_bias >= 0.85f * motor_max_curr) {
                                stop_scanning = true;
                            } else if (sat_lut_count >= SAT_MAX_LUT_POINTS) {
                                stop_scanning = true;
                            }

                            if (stop_scanning) {
                                if (sat_lut_count >= 3) {
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
                                        A = 1.0f / sat_lut_l[0];
                                        B = 0.0f;
                                    }

                                    if (B < 0.0f) {
                                        B = 0.0f;
                                        A = sum_y / n;
                                    }

                                    /* Extrapolate True L0 at 0 Amps with physical constraint */
                                    float L0_true = (A > 1e-6f) ? (1.0f / A) : sat_lut_l[0];

                                    float max_meas_L = sat_lut_l[0];
                                    if (L0_true < max_meas_L) L0_true = max_meas_L;
                                    if (L0_true > max_meas_L * 1.5f) L0_true = max_meas_L * 1.5f;

                                    id_result.sat_alpha = B * L0_true;
                                    id_result.sat_isat = (B > 1e-9f) ? sqrtf(1.0f / (B * L0_true))
                                                                     : motor_max_curr * 10.0f;
                                    id_result.measured_ls = L0_true;
                                    id_result.state = MOTOR_ID_STATE_COMPLETE;
                                } else if (sat_lut_count >= 1 && sat_L0_val > 1e-7f) {
                                    /* Graceful fallback: use linear anchor L0 when scan stops early
                                     */
                                    id_result.sat_alpha = 0.0f;
                                    id_result.sat_isat = motor_max_curr * 10.0f;
                                    id_result.measured_ls = sat_L0_val;
                                    id_result.state = MOTOR_ID_STATE_COMPLETE;
                                } else {
                                    id_result.error_code = 3;
                                    id_result.state = MOTOR_ID_STATE_ERROR;
                                }
                            } else {
                                sat_target_i_bias = next_i_bias;
                                sat_sub = 0;
                                sat_settle_cnt = 0;
                                sat_vd_error_int = 0.0f;
                            }
                        }
                    }
                } else {
                    if (sat_lut_count >= 1 && sat_L0_val > 1e-7f) {
                        /* High-current ceiling reached: finalize with collected L0 instead of
                         * aborting */
                        id_result.sat_alpha = 0.0f;
                        id_result.sat_isat = motor_max_curr * 10.0f;
                        id_result.measured_ls = sat_L0_val;
                        id_result.state = MOTOR_ID_STATE_COMPLETE;
                    } else {
                        id_result.error_code = 2;
                        id_result.state = MOTOR_ID_STATE_ERROR;
                    }
                }
            }
        }
    }
}

/*===========================================================================*/
/* Inertia ID State Machine (Called at 1kHz from Slow Task)                  */
/*===========================================================================*/

static uint8_t s_inertia_sub = 0;
static uint32_t s_inertia_timer = 0;
static uint32_t s_inertia_settle_cnt = 0;

static float s_inertia_w1 = 0.0f;
static float s_inertia_w2 = 0.0f;
static float s_inertia_w3 = 0.0f;
static float s_inertia_w4 = 0.0f;

static float s_inertia_sum_iq_up = 0.0f;
static float s_inertia_sum_iq_down = 0.0f;
static uint32_t s_inertia_n_up = 0;
static uint32_t s_inertia_n_down = 0;
static uint32_t s_inertia_n_target = 0;

static float s_inertia_w_start_up = 0.0f;
static float s_inertia_w_end_up = 0.0f;
static float s_inertia_w_start_down = 0.0f;
static float s_inertia_w_end_down = 0.0f;

void MotorID_MeasureInertiaOffline(void) {
    if (g_foc.status.state == FOC_STATE_IDLE) {
        if (!FOC_SetControlMode(FOC_MODE_SPEED)) {
            return;
        }
        id_result.state = MOTOR_ID_STATE_MEASURE_INERTIA;
        s_inertia_sub = 0;
        s_inertia_pending = 1;

        /* Force Speed Mode for Inertia Measurement */
        g_foc.cmd.speed_ref = 0.0f;
        g_foc.cmd.speed_ref_target = 0.0f;

        FOC_Start();
    }
}

void MotorID_InertiaSlowTask(void) {
    if (id_result.state != MOTOR_ID_STATE_MEASURE_INERTIA || !s_inertia_pending) {
        return;
    }

    if (g_foc.status.state != FOC_STATE_RUN) {
        return;
    }

    if (s_inertia_sub == 0) { /* INIT */
        float flux =
            id_result.measured_flux > 0.0f ? id_result.measured_flux : g_foc.cfg.motor_flux;
        float Kt = 1.5f * g_foc.cfg.motor_poles * flux;
        if (Kt < 0.0001f) Kt = 0.0001f;

        /* Physical scaling law for PMSM/BLDC motors:
         * b0 = (p * Kt) / J is roughly inversely proportional to Kt.
         * Safe adaptive initialization across micro to heavy-lift drone motors: */
        float b0_init = 35.0f / (Kt + 0.001f);
        if (b0_init < 50.0f) b0_init = 50.0f;
        if (b0_init > 50000.0f) b0_init = 50000.0f;

        float motor_max_curr = FlashConfig_Get()->motor_max_curr;
        if (motor_max_curr < 0.5f) motor_max_curr = 2.0f;

        LADRC_SetGains(&g_foc.ctrl.speed, g_foc.cfg.ladrc_omega_c, g_foc.cfg.ladrc_omega_o,
                       b0_init);
        LADRC_SetLimits(&g_foc.ctrl.speed, SPEED_LOOP_OUT_MIN, motor_max_curr);

        /* 4-threshold flying window configuration: w1 < w2 < w3 < w4 */
        float min_floor_elec = 300.0f * RPM_TO_RAD * g_foc.cfg.motor_poles;
        float w1 = fmaxf(2.0f * g_foc.cfg.motor_min_spd, min_floor_elec);
        float w4 = 2.5f * w1;
        if (g_foc.cfg.motor_max_spd > 0.0f && w4 > 0.45f * g_foc.cfg.motor_max_spd) {
            w4 = 0.45f * g_foc.cfg.motor_max_spd;
        }
        if (w4 < w1 + 300.0f * RPM_TO_RAD * g_foc.cfg.motor_poles) {
            w4 = w1 + 300.0f * RPM_TO_RAD * g_foc.cfg.motor_poles;
        }

        float delta_w = w4 - w1;
        s_inertia_w1 = w1;
        s_inertia_w2 = w1 + 0.25f * delta_w;
        s_inertia_w3 = w1 + 0.75f * delta_w;
        s_inertia_w4 = w4;

        g_foc.cmd.speed_ref_target = s_inertia_w1;

        s_saved_accel = g_foc.cfg.speed_ramp_accel;
        /* Symmetric 1.0s ramp rate for smooth acceleration and linear actuator region */
        g_foc.cfg.speed_ramp_accel = delta_w / 0.5f;

        s_inertia_sub = 1;
        s_inertia_timer = 0;
        s_inertia_settle_cnt = 0;
        s_inertia_sum_iq_up = 0.0f;
        s_inertia_sum_iq_down = 0.0f;
        s_inertia_n_up = 0;
        s_inertia_n_down = 0;
        s_inertia_n_target = 0;
        s_inertia_w_start_up = 0.0f;
        s_inertia_w_end_up = 0.0f;
        s_inertia_w_start_down = 0.0f;
        s_inertia_w_end_down = 0.0f;
    } else if (s_inertia_sub == 1) { /* SETTLE_W1 */
        g_foc.cmd.speed_ref_target = s_inertia_w1;
        s_inertia_timer++;

        float tol_min = 30.0f * RPM_TO_RAD * g_foc.cfg.motor_poles;
        float tol = 0.15f * s_inertia_w1;
        if (tol < tol_min) tol = tol_min;

        if (fabsf(g_foc.data.omega_elec - s_inertia_w1) < tol &&
            g_foc.cmd.speed_ref == g_foc.cmd.speed_ref_target) {
            s_inertia_settle_cnt++;
        }

        /* Wait 300 ms steady-state at w1 before initiating ramp */
        if (s_inertia_settle_cnt >= 300) {
            s_inertia_sub = 2;
            s_inertia_timer = 0;
            s_inertia_settle_cnt = 0;
            g_foc.cmd.speed_ref_target = s_inertia_w4;
        }

        if (s_inertia_timer > 10000) { /* 10s Timeout */
            g_foc.cfg.speed_ramp_accel = s_saved_accel;
            id_result.error_code = 7;
            id_result.state = MOTOR_ID_STATE_ERROR;
            FOC_Stop();
            s_inertia_pending = 0;
        }
    } else if (s_inertia_sub == 2) { /* RAMP_UP & WINDOW_UP */
        s_inertia_timer++;

        /* Flying window UP: sample strictly between w2 and w3 */
        if (g_foc.data.omega_elec >= s_inertia_w2 && g_foc.data.omega_elec <= s_inertia_w3) {
            if (s_inertia_n_up == 0) {
                s_inertia_w_start_up = g_foc.data.omega_elec;
            }
            s_inertia_sum_iq_up += g_foc.data.Iq;
            s_inertia_n_up++;
            s_inertia_w_end_up = g_foc.data.omega_elec;
        }

        /* Check if reference reached w4 and speed traversed past w3 */
        if (g_foc.cmd.speed_ref == g_foc.cmd.speed_ref_target &&
            g_foc.data.omega_elec >= s_inertia_w3 && s_inertia_n_up > 0) {
            s_inertia_sub = 3;
            s_inertia_timer = 0;
            s_inertia_n_target = s_inertia_n_up;
        }

        if (s_inertia_timer > 15000) { /* 15s Timeout */
            g_foc.cfg.speed_ramp_accel = s_saved_accel;
            id_result.error_code = 7;
            id_result.state = MOTOR_ID_STATE_ERROR;
            FOC_Stop();
            s_inertia_pending = 0;
        }
    } else if (s_inertia_sub == 3) { /* HOLD_W4 (Buffer zone) */
        s_inertia_timer++;

        /* Hold at w4 for 200 ms to decouple turnaround transient */
        if (s_inertia_timer >= 200) {
            s_inertia_sub = 4;
            s_inertia_timer = 0;
            g_foc.cmd.speed_ref_target = s_inertia_w1;
        }
    } else if (s_inertia_sub == 4) { /* RAMP_DOWN & WINDOW_DOWN */
        s_inertia_timer++;

        /* Flying window DOWN: start when speed drops to w3, collect exactly N_target samples */
        if (g_foc.data.omega_elec <= s_inertia_w3 && s_inertia_n_down < s_inertia_n_target) {
            if (s_inertia_n_down == 0) {
                s_inertia_w_start_down = g_foc.data.omega_elec;
            }
            s_inertia_sum_iq_down += g_foc.data.Iq;
            s_inertia_n_down++;
            s_inertia_w_end_down = g_foc.data.omega_elec;
        }

        /* Once exactly equal number of samples collected, proceed immediately */
        if (s_inertia_n_down >= s_inertia_n_target && s_inertia_n_target > 0) {
            s_inertia_sub = 5;
        }

        if (s_inertia_timer > 15000) { /* 15s Timeout */
            g_foc.cfg.speed_ramp_accel = s_saved_accel;
            id_result.error_code = 7;
            id_result.state = MOTOR_ID_STATE_ERROR;
            FOC_Stop();
            s_inertia_pending = 0;
        }
    } else if (s_inertia_sub == 5) { /* CALCULATE */
        float delta_w_up = s_inertia_w_end_up - s_inertia_w_start_up;
        float delta_w_down = s_inertia_w_start_down - s_inertia_w_end_down;
        float delta_w_elec = 0.5f * (delta_w_up + delta_w_down);

        /* Bidirectional net acceleration current integral (cancels all symmetric load & friction):
         * dt = 1ms = 0.001f */
        float net_I_integral = 0.5f * (s_inertia_sum_iq_up - s_inertia_sum_iq_down) * 0.001f;

        if (delta_w_elec > 1.0f && net_I_integral > 0.0001f && s_inertia_n_target > 0) {
            id_result.measured_b0 = delta_w_elec / net_I_integral;

            float flux =
                id_result.measured_flux > 0.0f ? id_result.measured_flux : g_foc.cfg.motor_flux;
            float Kt = 1.5f * g_foc.cfg.motor_poles * flux;

            /* J = (poles * Kt) / b0_elec */
            id_result.measured_inertia = (g_foc.cfg.motor_poles * Kt) / id_result.measured_b0;

            id_result.state = MOTOR_ID_STATE_COMPLETE;
        } else {
            id_result.error_code = 7;
            id_result.state = MOTOR_ID_STATE_ERROR;
        }

        g_foc.cfg.speed_ramp_accel = s_saved_accel;
        FOC_Stop();
        s_inertia_pending = 0;
        s_inertia_sub = 6;
    }
}

CCMRAM_FUNC uint8_t MotorID_IsRunning(void) {
    return ((id_result.state != MOTOR_ID_STATE_IDLE && id_result.state != MOTOR_ID_STATE_COMPLETE &&
             id_result.state != MOTOR_ID_STATE_ERROR) ||
            (g_foc.status.state == FOC_STATE_SELF_COMMISSION ||
             g_foc.status.state == FOC_STATE_COAST_FLUX_ID));
}