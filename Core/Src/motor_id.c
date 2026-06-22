/**
 * @file motor_id.c
 * @brief Motor Parameter Identification — Dual-Amplitude AC Injection with Adaptive Probing
 *
 * Implements an industry-standard parameter identification process:
 * 1. Rotor Alignment: Locks the rotor along the d-axis (theta_elec = 0)
 *    using d-axis current regulation to 5% of maximum current.
 * 2. Adaptive Probing: Injects a series of small-amplitude 1000 Hz AC voltages,
 *    measuring feedback current until it reaches the optimal SNR sweet spot (0.4A - 2.0A).
 * 3. AC1 Measurement: Injects AC at the determined sweet-spot amplitude (Vsweet),
 *    demodulates the current using synchronous demodulation (Lock-in/DFT).
 * 4. AC2 Measurement: Injects AC at double the sweet-spot amplitude (2 * Vsweet),
 *    demodulates the current.
 * 5. Parameter Extraction: Solves the two-point AC impedance equations to separate
 *    true phase resistance Rs from dead-time distortion. Computes true phase inductance Ls
 *    and estimates the physical inverter dead-time.
 */

#include "motor_id.h"

#include <math.h>

#include "cordic_math.h"
#include "foc_config.h"
#include "foc_state_machine.h"

/*===========================================================================*/
/* Configuration Constants                                                   */
/*===========================================================================*/

/* ---- Alignment ---- */
#define ID_ALIGN_TIME_MS 300

/* ---- Probing ---- */
#define ID_PROBE_I_MIN 0.4f    /* Minimum current amplitude for good SNR [A] */
#define ID_PROBE_I_MAX 2.0f    /* Maximum current amplitude for safety [A] */
#define ID_PROBE_V_INIT 0.10f  /* Initial probe voltage [V] */
#define ID_PROBE_V_STEP 0.15f  /* Probe voltage increment step [V] */
#define ID_PROBE_V_MAX 3.0f    /* Maximum safe probe voltage [V] */
#define ID_PROBE_CYC 10        /* Number of cycles to run per probe test */

/* ---- AC Injection ---- */
#define ID_LS_FREQ 1000.0f    /* Injection frequency [Hz] */
#define ID_LS_SKIP_CYC 5      /* Initial cycles to discard */
#define ID_LS_MEAS_CYC 20     /* Cycles to average */
#define ID_SETTLE_TIME_MS 100 /* Settle time before injection [ms] */

/* Derived constants */
#define ID_LS_OMEGA (TWO_PI * ID_LS_FREQ)
#define ID_LS_PHASE_INC (ID_LS_OMEGA * CONTROL_PERIOD_F) /* rad/sample */
#define ID_LS_SPR_CYC ((uint32_t)(PWM_FREQUENCY / ID_LS_FREQ))
#define ID_LS_SKIP_SAMP (ID_LS_SKIP_CYC * ID_LS_SPR_CYC)
#define ID_LS_TOTAL_SAMP ((ID_LS_SKIP_CYC + ID_LS_MEAS_CYC) * ID_LS_SPR_CYC)

/*===========================================================================*/
/* Private State                                                             */
/*===========================================================================*/

MotorID_Result_t id_result;

/* Timekeeping */
static uint32_t id_timer_ms;
static uint32_t tick_counter;

/* Current filter state */
static float id_filt;

/* AC Integration values */
static float ls_theta;
static uint32_t ls_sample_count;
static float ls_sum_I_sin;
static float ls_sum_I_cos;
static uint32_t ls_meas_count;

/* Probing variables */
static float Vprobe;
static float Vsweet;
static float align_vd;

/* Intermediate measurement storage */
static float Rapp1, Ls1, Iamp1;
static float Rapp2, Ls2, Iamp2;

static MotorID_State_t prev_state;

/*===========================================================================*/
/* Public API                                                                */
/*===========================================================================*/

void MotorID_Init(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
    prev_state = MOTOR_ID_STATE_IDLE;
    id_result.measured_rs = 0.0f;
    id_result.measured_ls = 0.0f;
    id_result.error_code = 0;
    id_result.identified_v_err = 0.0f;
    id_result.identified_deadtime_ns = 0.0f;
    id_result.dbg_ac1_Iamp = 0.0f;
    id_result.dbg_ac1_Rapp = 0.0f;
    id_result.dbg_ac1_Ls = 0.0f;
    id_result.dbg_ac2_Iamp = 0.0f;
    id_result.dbg_ac2_Rapp = 0.0f;
    id_result.dbg_ac2_Ls = 0.0f;
}

void MotorID_Start(void) {
    id_result.state = MOTOR_ID_STATE_ALIGN;
    prev_state = MOTOR_ID_STATE_IDLE;
    id_result.error_code = 0;
    id_timer_ms = 0;
    tick_counter = 0;
    id_filt = 0.0f;
    
    ls_theta = 0.0f;
    ls_sample_count = 0;
    ls_sum_I_sin = 0.0f;
    ls_sum_I_cos = 0.0f;
    ls_meas_count = 0;
    
    Vprobe = ID_PROBE_V_INIT;
    Vsweet = ID_PROBE_V_INIT;
    align_vd = 0.0f;

    Rapp1 = Ls1 = Iamp1 = 0.0f;
    Rapp2 = Ls2 = Iamp2 = 0.0f;
    
    id_result.measured_rs = 0.0f;
    id_result.measured_ls = 0.0f;
    id_result.identified_v_err = 0.0f;
    id_result.identified_deadtime_ns = 0.0f;
    id_result.dbg_ac1_Iamp = 0.0f;
    id_result.dbg_ac1_Rapp = 0.0f;
    id_result.dbg_ac1_Ls = 0.0f;
    id_result.dbg_ac2_Iamp = 0.0f;
    id_result.dbg_ac2_Rapp = 0.0f;
    id_result.dbg_ac2_Ls = 0.0f;
}

void MotorID_Stop(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
}

void MotorID_RunStep(float id, float iq, float vbus, float* vd, float* vq) {
    (void)iq;  // Avoid unused parameter warning

    /* Timekeeping (1 ms tick) */
    if (++tick_counter >= (PWM_FREQUENCY / 1000)) {
        id_timer_ms++;
        tick_counter = 0;
    }

    /* Apply low-pass filter to Id current for feedback and monitoring */
    id_filt += CURRENT_FILTER_COEFF * (id - id_filt);

    /* Driver state control */
    if (id_result.state != prev_state) {
        switch (id_result.state) {
            case MOTOR_ID_STATE_ALIGN:
            case MOTOR_ID_STATE_PROBE:
            case MOTOR_ID_STATE_MEASURE_AC1:
            case MOTOR_ID_STATE_MEASURE_AC2:
                FOC_EnableDrivers(1);
                break;
            default:
                FOC_EnableDrivers(0);
                break;
        }
        prev_state = id_result.state;
    }

    /* Default values */
    *vd = 0.0f;
    *vq = 0.0f;

    switch (id_result.state) {
        case MOTOR_ID_STATE_IDLE:
        case MOTOR_ID_STATE_COMPLETE:
            *vd = 0.0f;
            *vq = 0.0f;
            break;

        case MOTOR_ID_STATE_ERROR:
            FOC_EnableDrivers(0);
            *vd = 0.0f;
            *vq = 0.0f;
            break;

        /* ================================================================== */
        /* ALIGNMENT STAGE: Lock rotor along d-axis (theta_elec = 0)          */
        /* ================================================================== */
        case MOTOR_ID_STATE_ALIGN: {
            /* Regulate Id current to 5% of maximum phase current */
            float target_i = 0.05f * g_foc.cfg.motor_max_curr;
            float err = target_i - id_filt;
            const float Ki_align = 0.0002f; // Slow, stable integration gain

            align_vd += Ki_align * err;

            /* Safety: limit alignment voltage to 30% of Vbus */
            float max_v = 0.30f * vbus;
            if (align_vd < 0.0f) {
                align_vd = 0.0f;
            } else if (align_vd > max_v) {
                align_vd = max_v;
            }

            *vd = align_vd;
            *vq = 0.0f;

            if (id_timer_ms >= ID_ALIGN_TIME_MS) {
                id_result.state = MOTOR_ID_STATE_PROBE;
                id_timer_ms = 0;
                tick_counter = 0;
                
                /* Reset AC variables for Probing */
                Vprobe = ID_PROBE_V_INIT;
                ls_theta = 0.0f;
                ls_sample_count = 0;
                ls_sum_I_sin = 0.0f;
                ls_sum_I_cos = 0.0f;
                ls_meas_count = 0;
            }
        } break;

        /* ================================================================== */
        /* ADAPTIVE PROBING STAGE: Find optimal sine voltage amplitude        */
        /* ================================================================== */
        case MOTOR_ID_STATE_PROBE: {
            /* Settle time before injection */
            if (id_timer_ms < ID_SETTLE_TIME_MS) {
                *vd = 0.0f;
                *vq = 0.0f;
                break;
            }

            /* Phase accumulation */
            ls_theta += ID_LS_PHASE_INC;
            if (ls_theta >= PI) {
                ls_theta -= TWO_PI;
            }

            float s, c;
            cordic_sincos(ls_theta / PI, &c, &s);

            *vd = Vprobe * s;
            *vq = 0.0f;

            ls_sample_count++;

            /* Demodulate current in steady-state */
            if (ls_sample_count > ID_LS_SKIP_SAMP) {
                ls_sum_I_sin += id * s;
                ls_sum_I_cos += id * c;
                ls_meas_count++;
            }

            /* Check amplitude after skips + probe cycles */
            uint32_t probe_samples = (ID_LS_SKIP_CYC + ID_PROBE_CYC) * ID_LS_SPR_CYC;
            if (ls_sample_count >= probe_samples) {
                float inv_N = 1.0f / (float)ls_meas_count;
                float I_re = 2.0f * ls_sum_I_sin * inv_N;
                float I_im = -2.0f * ls_sum_I_cos * inv_N;
                float I_amp = sqrtf(I_re * I_re + I_im * I_im);

                if (I_amp >= ID_PROBE_I_MIN && I_amp <= ID_PROBE_I_MAX) {
                    /* Optimal current range reached, set Vsweet */
                    Vsweet = Vprobe;
                    id_result.state = MOTOR_ID_STATE_MEASURE_AC1;
                    id_timer_ms = 0;
                    tick_counter = 0;

                    /* Reset variables for AC1 */
                    ls_theta = 0.0f;
                    ls_sample_count = 0;
                    ls_sum_I_sin = 0.0f;
                    ls_sum_I_cos = 0.0f;
                    ls_meas_count = 0;
                } else if (I_amp > ID_PROBE_I_MAX) {
                    /* Current is already slightly too high, use current Vprobe as Vsweet */
                    Vsweet = Vprobe;
                    id_result.state = MOTOR_ID_STATE_MEASURE_AC1;
                    id_timer_ms = 0;
                    tick_counter = 0;

                    /* Reset variables for AC1 */
                    ls_theta = 0.0f;
                    ls_sample_count = 0;
                    ls_sum_I_sin = 0.0f;
                    ls_sum_I_cos = 0.0f;
                    ls_meas_count = 0;
                } else {
                    /* Current is too low, increase Vprobe and try again */
                    Vprobe += ID_PROBE_V_STEP;
                    if (Vprobe > ID_PROBE_V_MAX || Vprobe > 0.35f * vbus) {
                        /* Exceeded safety voltage limit, abort */
                        id_result.error_code = 2; // Low current / disconnected motor
                        id_result.state = MOTOR_ID_STATE_ERROR;
                    } else {
                        /* Re-run probe test with the higher voltage */
                        id_timer_ms = 0;
                        tick_counter = 0;
                        ls_theta = 0.0f;
                        ls_sample_count = 0;
                        ls_sum_I_sin = 0.0f;
                        ls_sum_I_cos = 0.0f;
                        ls_meas_count = 0;
                    }
                }
            }
        } break;

        /* ================================================================== */
        /* MEASURE AC1: Low Amplitude AC Injection (Vsweet)                  */
        /* ================================================================== */
        case MOTOR_ID_STATE_MEASURE_AC1: {
            /* Settle time before injection */
            if (id_timer_ms < ID_SETTLE_TIME_MS) {
                *vd = 0.0f;
                *vq = 0.0f;
                break;
            }

            /* Phase accumulation */
            ls_theta += ID_LS_PHASE_INC;
            if (ls_theta >= PI) {
                ls_theta -= TWO_PI;
            }

            float s, c;
            cordic_sincos(ls_theta / PI, &c, &s);

            *vd = Vsweet * s;
            *vq = 0.0f;

            ls_sample_count++;

            /* Accumulate only in steady-state */
            if (ls_sample_count > ID_LS_SKIP_SAMP) {
                ls_sum_I_sin += id * s;
                ls_sum_I_cos += id * c;
                ls_meas_count++;
            }

            if (ls_sample_count >= ID_LS_TOTAL_SAMP) {
                float inv_N = 1.0f / (float)ls_meas_count;
                float I_re = 2.0f * ls_sum_I_sin * inv_N;
                float I_im = -2.0f * ls_sum_I_cos * inv_N;
                float I_amp_sq = I_re * I_re + I_im * I_im;

                if (I_amp_sq > 1e-4f) {
                    Iamp1 = sqrtf(I_amp_sq);
                    Rapp1 = (Vsweet * I_re) / I_amp_sq;
                    Ls1 = (Vsweet * I_im) / (ID_LS_OMEGA * I_amp_sq);

                    id_result.dbg_ac1_Iamp = Iamp1;
                    id_result.dbg_ac1_Rapp = Rapp1;
                    id_result.dbg_ac1_Ls = Ls1;

                    /* Move to AC2 stage */
                    id_result.state = MOTOR_ID_STATE_MEASURE_AC2;
                    id_timer_ms = 0;
                    tick_counter = 0;
                    
                    /* Reset AC variables */
                    ls_theta = 0.0f;
                    ls_sample_count = 0;
                    ls_sum_I_sin = 0.0f;
                    ls_sum_I_cos = 0.0f;
                    ls_meas_count = 0;
                } else {
                    id_result.error_code = 2; // Low current amplitude
                    id_result.state = MOTOR_ID_STATE_ERROR;
                }
            }
        } break;

        /* ================================================================== */
        /* MEASURE AC2: High Amplitude AC Injection (2 * Vsweet)             */
        /* ================================================================== */
        case MOTOR_ID_STATE_MEASURE_AC2: {
            /* Settle time before injection */
            if (id_timer_ms < ID_SETTLE_TIME_MS) {
                *vd = 0.0f;
                *vq = 0.0f;
                break;
            }

            /* Phase accumulation */
            ls_theta += ID_LS_PHASE_INC;
            if (ls_theta >= PI) {
                ls_theta -= TWO_PI;
            }

            float s, c;
            cordic_sincos(ls_theta / PI, &c, &s);

            *vd = (2.0f * Vsweet) * s;
            *vq = 0.0f;

            ls_sample_count++;

            /* Accumulate only in steady-state */
            if (ls_sample_count > ID_LS_SKIP_SAMP) {
                ls_sum_I_sin += id * s;
                ls_sum_I_cos += id * c;
                ls_meas_count++;
            }

            if (ls_sample_count >= ID_LS_TOTAL_SAMP) {
                float inv_N = 1.0f / (float)ls_meas_count;
                float I_re = 2.0f * ls_sum_I_sin * inv_N;
                float I_im = -2.0f * ls_sum_I_cos * inv_N;
                float I_amp_sq = I_re * I_re + I_im * I_im;

                if (I_amp_sq > 1e-4f) {
                    Iamp2 = sqrtf(I_amp_sq);
                    Rapp2 = ((2.0f * Vsweet) * I_re) / I_amp_sq;
                    Ls2 = ((2.0f * Vsweet) * I_im) / (ID_LS_OMEGA * I_amp_sq);

                    id_result.dbg_ac2_Iamp = Iamp2;
                    id_result.dbg_ac2_Rapp = Rapp2;
                    id_result.dbg_ac2_Ls = Ls2;

                    /* Calculate real values by combining two AC points */
                    if (Iamp2 > Iamp1 && Rapp1 > Rapp2) {
                        float rs = (Iamp2 * Rapp2 - Iamp1 * Rapp1) / (Iamp2 - Iamp1);
                        float ls = Ls2; // Inductance measured at high current (better SNR)
                        
                        /* Calculate dead-time voltage error */
                        float inv_i_diff = 1.0f / Iamp1 - 1.0f / Iamp2;
                        float v_err = 0.785398163f * (Rapp1 - Rapp2) / inv_i_diff;

                        /* Convert V_err to physical dead-time (ns) */
                        float td_ns = (v_err / vbus) * (1e9f / (float)PWM_FREQUENCY);

                        if (rs > 0.0f && ls > 0.0f && v_err > 0.0f) {
                            id_result.measured_rs = rs;
                            id_result.measured_ls = ls;
                            id_result.identified_v_err = v_err;
                            id_result.identified_deadtime_ns = td_ns;

                            id_result.state = MOTOR_ID_STATE_COMPLETE;
                        } else {
                            id_result.error_code = 3; // Parameter bounds violation
                            id_result.state = MOTOR_ID_STATE_ERROR;
                        }
                    } else {
                        id_result.error_code = 3; // Inconsistent AC measurement results
                        id_result.state = MOTOR_ID_STATE_ERROR;
                    }
                } else {
                    id_result.error_code = 2; // AC2 current amplitude error
                    id_result.state = MOTOR_ID_STATE_ERROR;
                }
            }
        } break;
    }
}

void MotorID_GetResults(MotorID_Result_t* results) {
    if (results) {
        *results = id_result;
    }
}
