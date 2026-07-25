/**
 * @file motor_id.c
 * @brief Motor Parameter Identification — PI-DC Rs + Dual-Frequency AC Ls
 *
 * ALGORITHM OVERVIEW
 * ──────────────────
 *
 * 1. ALIGN (300 ms)
 *    Slow integral controller ramps Vd to lock the rotor along the d-axis
 *    at a small holding current (5 % of I_max). theta_elec = 0 throughout.
 *
 * 2. MEASURE_RS — PI-controlled DC 2-point injection
 *    With the rotor locked and pure DC (no AC), the inductor is a short:
 *        Vd = Rs * Id + Vdead(sign(Id))
 *    Vdead is CONSTANT for Id > 0.  Two operating points:
 *        Vd1 = Rs * I1 + Vdead
 *        Vd2 = Rs * I2 + Vdead
 *        → Rs = (Vd2 − Vd1) / (I2 − I1)   [dead-time cancels!]
 *    A pure integrator (no P) controls Vd until Id settles.
 *    Vd is averaged over a measurement window for noise rejection.
 *
 * 3. FREQ_DETECT — Frequency pair selection
 *    Quick AC probe at 1000 Hz, 10 cycles, fixed V_probe = 1 V.
 *    The raw impedance angle φ_app = arctan(ω·Ls_raw / Rapp_raw) is computed
 *    (delay-ignorant; good enough for frequency selection).
 *    From φ_app the motor corner frequency f_corner = R/(2π·L) is estimated,
 *    then the target f1 is chosen to give φ_m1 ≈ 25° (optimal for k=4):
 *        f1_ideal = f_corner × tan(25°) = f_corner × 0.4663
 *    f1 is rounded to the nearest candidate in {250, 500, 1000, 2000} Hz.
 *    f2 = 4 × f1 if f2 ≤ 4000 Hz, else f2 = 2 × f1.
 *
 * 4. MEASURE_LS_F1  /  MEASURE_LS_F2 — AC lock-in at f1 and f2
 *    Each phase: probe sweep to find Vsweet (optimal injection amplitude),
 *    then 25-cycle synchronous demodulation (lock-in).
 *    RAW phasors (I_re_raw, I_im_raw) are stored WITHOUT delay compensation.
 *    Rapp_raw and Ls_raw are derived from the raw phasors.
 *
 * 5. EXTRACT — Self-calibrating delay + final Ls
 *    The corrected Ls as a function of the unknown delay N (samples):
 *        Ls_c(f, N) = Ls_raw · cos(φ) − (Rapp_raw/ω) · sin(φ),
 *        where φ = ω·N/Fs
 *    At the true hardware delay N*, both frequencies give the same Ls:
 *        f(N) = Ls_c(f1, N) − Ls_c(f2, N) = 0   at N = N*
 *    The scan minimises |f(N)| over N ∈ [0.0, 5.0] step 0.05 (100 points).
 *    Result: N* → id_result.identified_delay_samples
 *            Ls → id_result.measured_ls
 *    If the minimum is not well-defined (Δf flat → high-L/R motor at low freq),
 *    the raw Ls at f1 with a fixed N = 0.5 is used as a safe fallback.
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
#define ID_ALIGN_KI 0.0002f /* Integrator gain [V/(A·sample)] */

/* ---- Rs Measurement ---- */
#define ID_RS_KI 0.0001f    /* Pure-I controller gain [V/(A·sample)] */
#define ID_RS_I1_FRAC 0.05f /* First current point: 5% of I_max  [–] */
#define ID_RS_I2_FRAC 0.20f /* Second current point: 20% of I_max [–] */
#define ID_RS_SETTLE_MS 150 /* Time to wait for current to settle  [ms] */
#define ID_RS_MEAS_MS 80    /* Averaging window for Vd and Id      [ms] */

/* ---- Frequency Detection ---- */
#define ID_FD_PROBE_HZ 1000.0f /* Base probe frequency                [Hz] */
#define ID_FD_V_PROBE 1.0f     /* Fixed probe voltage during detect    [V]  */
#define ID_FD_SKIP_CYC 2       /* Skip initial cycles for settle             */
#define ID_FD_MEAS_CYC 8       /* Measurement cycles for freq detect         */

/* ---- Ls Probe Sweep ---- */
#define ID_LS_PROBE_V_INIT 1.0f /* Initial probe voltage               [V]   */
#define ID_LS_PROBE_V_STEP 0.5f /* Probe voltage increment             [V]   */
#define ID_LS_PROBE_V_MAX 10.0f /* Hard limit on probe voltage         [V]   */
#define ID_LS_PROBE_I_MIN 0.40f /* Minimum acceptable I_amp            [A]   */
#define ID_LS_PROBE_I_MAX 4.50f /* Maximum acceptable I_amp            [A]   */
#define ID_LS_PROBE_CYC 5       /* Cycles per probe attempt                   */

/* ---- Ls Measurement ---- */
#define ID_LS_SKIP_CYC 8   /* Skip cycles for settle                     */
#define ID_LS_MEAS_CYC 25  /* Measurement averaging cycles               */
#define ID_LS_SETTLE_MS 80 /* Settle time before injection (Vd=0) [ms]  */

/* ---- Extract ---- */
#define ID_EXTRACT_N_MIN 0.0f
#define ID_EXTRACT_N_MAX 5.0f
#define ID_EXTRACT_N_STEP 0.05f /* 100 points total                          */
#define ID_EXTRACT_PER_ISR 10   /* Points processed per ISR call             */

/* ---- Candidate frequency list (≤ 4000 Hz for aliasing reasons) ---- */
static const float id_ls_candidates[] = {250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f};
#define ID_LS_NCANDIDATES 5

/*===========================================================================*/
/* Private State                                                             */
/*===========================================================================*/

MotorID_Result_t id_result;

/* Timekeeping */
static uint32_t id_timer_ms;
static uint32_t tick_counter;

/* Current low-pass filter */
static float id_filt;

/* Previous state (for driver enable edge detection) */
static MotorID_State_t prev_state;

/* ── ALIGN ── */
static float align_vd;

/* ── MEASURE_RS ── */
/* Sub-states: 0=SETTLE_I1, 1=MEAS_I1, 2=SETTLE_I2, 3=MEAS_I2, 4=COMPUTE */
static uint8_t rs_sub;
static float rs_vd_integrator;
static float rs_id_target;
static double rs_vd_sum; /* double for accuracy on long sum */
static double rs_id_sum;
static uint32_t rs_sum_count;
static float rs_vd1, rs_id1; /* Stored point 1 */
static float rs_vbus_avg;    /* Vbus averaged during Rs measurement */

/* ── FREQ_DETECT ── */
static float    fd_vprobe;       /* Adaptive probe voltage [V] */
static float    fd_theta;
static float    fd_phase_inc;
static uint32_t fd_sample_count;
static float    fd_sum_sin;
static float    fd_sum_cos;
static uint32_t fd_meas_count;
static uint32_t fd_skip_samp;
static uint32_t fd_total_samp;

/* ── MEASURE_LS (shared between F1 and F2) ── */
/* Sub-states: 0=SETTLE, 1=PROBE, 2=MEASURE */
static uint8_t ls_sub;
static float ls_freq;
static float ls_omega;
static float ls_phase_inc;
static uint32_t ls_spr; /* Samples per revolution at ls_freq */
static float ls_vprobe;
static float ls_vsweet;
static float ls_theta;
static uint32_t ls_sample_count;
static float ls_sum_sin;
static float ls_sum_cos;
static uint32_t ls_meas_count;
static uint32_t ls_skip_samp;
static uint32_t ls_total_samp;
static uint32_t ls_probe_samp;

/* Stored raw phasors for EXTRACT */
static float rapp_raw_f1, ls_raw_f1, iamp_f1;
static float rapp_raw_f2, ls_raw_f2, iamp_f2;
static float omega_f1, omega_f2;

/* ── EXTRACT ── */
static int ext_idx; /* Current scan index (0..99) */
static float ext_best_N;
static float ext_best_res; /* Smallest |Ls_c1 - Ls_c2| seen so far */
static float ext_res_at_0; /* Residual at N=0 (for convergence check) */

/*===========================================================================*/
/* Private Helpers                                                           */
/*===========================================================================*/

/** Update all per-frequency derived parameters */
static void ls_update_freq_params(float freq) {
    ls_freq = freq;
    ls_omega = TWO_PI * freq;
    ls_phase_inc = TWO_PI * freq * CONTROL_PERIOD_F;
    ls_spr = (uint32_t)(PWM_FREQUENCY / freq + 0.5f);
    ls_skip_samp = ID_LS_SKIP_CYC * ls_spr;
    ls_probe_samp = (ID_LS_SKIP_CYC + ID_LS_PROBE_CYC) * ls_spr;
    ls_total_samp = (ID_LS_SKIP_CYC + ID_LS_MEAS_CYC) * ls_spr;
}

/** Reset synchronous demodulator accumulators */
static void ls_reset_lockin(void) {
    ls_theta = 0.0f;
    ls_sample_count = 0;
    ls_sum_sin = 0.0f;
    ls_sum_cos = 0.0f;
    ls_meas_count = 0;
}

/**
 * Find the candidate frequency (from id_ls_candidates[]) nearest to f_target
 * on a logarithmic scale.
 */
static float find_nearest_candidate(float f_target) {
    float best = id_ls_candidates[0];
    /* Compare ratio log2(f/best): smallest absolute log-ratio wins */
    float best_ratio = f_target / best;
    if (best_ratio < 1.0f) best_ratio = 1.0f / best_ratio;

    for (int i = 1; i < ID_LS_NCANDIDATES; i++) {
        float ratio = f_target / id_ls_candidates[i];
        if (ratio < 1.0f) ratio = 1.0f / ratio;
        if (ratio < best_ratio) {
            best_ratio = ratio;
            best = id_ls_candidates[i];
        }
    }
    return best;
}

/** Return 1 if f is present in the candidate list (within 1 Hz tolerance) */
static int is_candidate(float f) {
    for (int i = 0; i < ID_LS_NCANDIDATES; i++) {
        if (fabsf(f - id_ls_candidates[i]) < 1.0f) return 1;
    }
    return 0;
}

/** Reset all timekeeping (call when entering a new phase) */
static void reset_timer(void) {
    id_timer_ms = 0;
    tick_counter = 0;
}

/*===========================================================================*/
/* Public API                                                                */
/*===========================================================================*/

void MotorID_Init(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
    id_result.measured_rs = 0.0f;
    id_result.measured_ls = 0.0f;
    id_result.error_code = 0;
    id_result.identified_v_err = 0.0f;
    id_result.identified_deadtime_ns = 0.0f;
    id_result.identified_delay_samples = 0.0f;
    id_result.dbg_f1_hz = 0.0f;
    id_result.dbg_f2_hz = 0.0f;
    id_result.dbg_phi_detect_deg = 0.0f;
    id_result.dbg_rapp_raw_f1 = 0.0f;
    id_result.dbg_ls_raw_f1 = 0.0f;
    id_result.dbg_iamp_f1 = 0.0f;
    id_result.dbg_rapp_raw_f2 = 0.0f;
    id_result.dbg_ls_raw_f2 = 0.0f;
    id_result.dbg_iamp_f2 = 0.0f;
    id_result.dbg_ls_comp_f1 = 0.0f;
    id_result.dbg_ls_comp_f2 = 0.0f;
    prev_state = MOTOR_ID_STATE_IDLE;
}

void MotorID_Start(void) {
    MotorID_Init();
    align_vd = 0.0f;
    id_filt = 0.0f;
    reset_timer();
    id_result.state = MOTOR_ID_STATE_ALIGN;
    prev_state = MOTOR_ID_STATE_IDLE;
}

void MotorID_Stop(void) {
    id_result.state = MOTOR_ID_STATE_IDLE;
}

void MotorID_GetResults(MotorID_Result_t* results) {
    if (results) *results = id_result;
}

/*===========================================================================*/
/* Main State Machine                                                        */
/*===========================================================================*/

void MotorID_RunStep(float id, float iq, float vbus, float* vd, float* vq) {
    (void)iq;

    /* 1-ms tick counter */
    if (++tick_counter >= (PWM_FREQUENCY / 1000)) {
        id_timer_ms++;
        tick_counter = 0;
    }

    /* Filtered current for monitoring */
    id_filt += CURRENT_FILTER_COEFF * (id - id_filt);

    /* Driver enable edge detection */
    if (id_result.state != prev_state) {
        int drivers_on = (id_result.state == MOTOR_ID_STATE_ALIGN ||
                          id_result.state == MOTOR_ID_STATE_MEASURE_RS ||
                          id_result.state == MOTOR_ID_STATE_FREQ_DETECT ||
                          id_result.state == MOTOR_ID_STATE_MEASURE_LS_F1 ||
                          id_result.state == MOTOR_ID_STATE_MEASURE_LS_F2);
        FOC_EnableDrivers(drivers_on ? 1 : 0);
        prev_state = id_result.state;
    }

    *vd = 0.0f;
    *vq = 0.0f;

    switch (id_result.state) {
        /* ================================================================== */
        case MOTOR_ID_STATE_IDLE:
        case MOTOR_ID_STATE_COMPLETE:
            break;

        /* ================================================================== */
        case MOTOR_ID_STATE_ERROR:
            FOC_EnableDrivers(0);
            break;

        /* ================================================================== */
        /* ALIGN: Slow integrator ramps Id to 5% I_max over 300 ms           */
        /* ================================================================== */
        case MOTOR_ID_STATE_ALIGN: {
            float target = ID_RS_I1_FRAC * g_foc.cfg.motor_max_curr;
            float err = target - id_filt;
            align_vd += ID_ALIGN_KI * err;

            /* Anti-windup: clamp to [0, 30% Vbus] */
            float v_max = 0.30f * vbus;
            if (align_vd < 0.0f)
                align_vd = 0.0f;
            else if (align_vd > v_max)
                align_vd = v_max;

            *vd = align_vd;
            *vq = 0.0f;

            if (id_timer_ms >= ID_ALIGN_TIME_MS) {
                /* Enter MEASURE_RS */
                rs_sub = 0;
                rs_vd_integrator = align_vd; /* Start from alignment voltage */
                rs_id_target = ID_RS_I1_FRAC * g_foc.cfg.motor_max_curr;
                rs_vd_sum = 0.0;
                rs_id_sum = 0.0;
                rs_sum_count = 0;
                rs_vbus_avg = vbus;
                reset_timer();
                id_result.state = MOTOR_ID_STATE_MEASURE_RS;
            }
        } break;

        /* ================================================================== */
        /* MEASURE_RS: Pure-I controller, 2-point DC measurement              */
        /*                                                                    */
        /*  Sub 0: Settle at I1 (ID_RS_SETTLE_MS)                            */
        /*  Sub 1: Average Vd and Id over ID_RS_MEAS_MS → (Vd1, Id1)        */
        /*  Sub 2: Switch target to I2, settle (ID_RS_SETTLE_MS)             */
        /*  Sub 3: Average → (Vd2, Id2)                                     */
        /*  Sub 4: Compute Rs = (Vd2-Vd1)/(Id2-Id1)                         */
        /* ================================================================== */
        case MOTOR_ID_STATE_MEASURE_RS: {
            /* Pure integrator: no P term — guaranteed convergence, no overshoot */
            float err = rs_id_target - id;
            rs_vd_integrator += ID_RS_KI * err;

            /* Anti-windup: clamp to [0, 45% Vbus] */
            float v_max = 0.45f * vbus;
            if (rs_vd_integrator < 0.0f)
                rs_vd_integrator = 0.0f;
            else if (rs_vd_integrator > v_max)
                rs_vd_integrator = v_max;

            *vd = rs_vd_integrator;
            *vq = 0.0f;

            const uint32_t settle_ms = ID_RS_SETTLE_MS;
            const uint32_t meas_ms = ID_RS_MEAS_MS;

            switch (rs_sub) {
                case 0: /* Settle at I1 */
                    if (id_timer_ms >= settle_ms) {
                        rs_vd_sum = 0.0;
                        rs_id_sum = 0.0;
                        rs_sum_count = 0;
                        rs_sub = 1;
                        reset_timer();
                    }
                    break;

                case 1: /* Measure at I1 */
                    rs_vd_sum += rs_vd_integrator;
                    rs_id_sum += id;
                    rs_sum_count++;
                    rs_vbus_avg += (vbus - rs_vbus_avg) * 0.001f; /* slow LP filter */

                    if (id_timer_ms >= meas_ms) {
                        rs_vd1 = (float)(rs_vd_sum / rs_sum_count);
                        rs_id1 = (float)(rs_id_sum / rs_sum_count);
                        /* Switch to I2 */
                        rs_id_target = ID_RS_I2_FRAC * g_foc.cfg.motor_max_curr;
                        rs_vd_sum = 0.0;
                        rs_id_sum = 0.0;
                        rs_sum_count = 0;
                        rs_sub = 2;
                        reset_timer();
                    }
                    break;

                case 2: /* Settle at I2 */
                    if (id_timer_ms >= settle_ms) {
                        rs_vd_sum = 0.0;
                        rs_id_sum = 0.0;
                        rs_sum_count = 0;
                        rs_sub = 3;
                        reset_timer();
                    }
                    break;

                case 3: /* Measure at I2 */
                    rs_vd_sum += rs_vd_integrator;
                    rs_id_sum += id;
                    rs_sum_count++;

                    if (id_timer_ms >= meas_ms) {
                        float vd2 = (float)(rs_vd_sum / rs_sum_count);
                        float id2 = (float)(rs_id_sum / rs_sum_count);

                        float delta_v = vd2 - rs_vd1;
                        float delta_i = id2 - rs_id1;

                        if (delta_i > 0.02f) {
                            float rs = delta_v / delta_i;
                            if (rs > 0.005f && rs < 20.0f) {
                                id_result.measured_rs = rs;
                                /* Enter FREQ_DETECT */
                                fd_vprobe = ID_LS_PROBE_V_INIT;
                                fd_theta = 0.0f;
                                fd_phase_inc = TWO_PI * ID_FD_PROBE_HZ * CONTROL_PERIOD_F;
                                uint32_t spr = (uint32_t)(PWM_FREQUENCY / ID_FD_PROBE_HZ + 0.5f);
                                fd_skip_samp = ID_FD_SKIP_CYC * spr;
                                fd_total_samp = (ID_FD_SKIP_CYC + ID_FD_MEAS_CYC) * spr;
                                fd_sample_count = 0;
                                fd_sum_sin = 0.0f;
                                fd_sum_cos = 0.0f;
                                fd_meas_count = 0;
                                reset_timer();
                                id_result.state = MOTOR_ID_STATE_FREQ_DETECT;
                            } else {
                                id_result.error_code = 1; /* Rs out of [0.005, 20] Ω */
                                id_result.state = MOTOR_ID_STATE_ERROR;
                            }
                        } else {
                            /* Delta current too small — integrator did not converge */
                            id_result.error_code = 7;
                            id_result.state = MOTOR_ID_STATE_ERROR;
                        }
                        reset_timer();
                    }
                    break;
            }
        } break;

        /* ================================================================== */
        /* FREQ_DETECT: Adaptive probe at 1000 Hz, select f1 and f2          */
        /*                                                                    */
        /* Uses adaptive voltage sweep (same as LS probe) to guarantee       */
        /* adequate SNR before computing phi. A fixed low voltage (1 V)      */
        /* would give I_amp ≈ 0.11 A for high-impedance motors (e.g. Nidec) */
        /* — near the ADC noise floor — making phi completely unreliable.    */
        /* ================================================================== */
        case MOTOR_ID_STATE_FREQ_DETECT: {
            /* 50 ms settle: let residual DC from MEASURE_RS decay to zero.
             * For Nidec: τ = L/R = 0.75 ms → after 50 ms essentially zero. */
            if (id_timer_ms < 50) {
                *vd = 0.0f;
                *vq = 0.0f;
                break;
            }

            /* Adaptive phase accumulation */
            fd_theta += fd_phase_inc;
            if (fd_theta >= PI) fd_theta -= TWO_PI;

            float s, c;
            cordic_sincos(fd_theta / PI, &c, &s);

            *vd = fd_vprobe * s;
            *vq = 0.0f;

            fd_sample_count++;

            if (fd_sample_count > fd_skip_samp) {
                fd_sum_sin += id * s;
                fd_sum_cos += id * c;
                fd_meas_count++;
            }

            if (fd_sample_count >= fd_total_samp) {
                float inv_N  = 1.0f / (float)fd_meas_count;
                float I_re   = 2.0f * fd_sum_sin * inv_N;
                float I_im   = -2.0f * fd_sum_cos * inv_N;
                float I_amp_sq = I_re * I_re + I_im * I_im;
                float I_amp  = sqrtf(I_amp_sq);

                if (I_amp < ID_LS_PROBE_I_MIN) {
                    /* Current too low → increase probe voltage and retry */
                    fd_vprobe += ID_LS_PROBE_V_STEP;
                    float v_safe = 0.45f * vbus;
                    if (v_safe > ID_LS_PROBE_V_MAX) v_safe = ID_LS_PROBE_V_MAX;
                    if (fd_vprobe > v_safe) fd_vprobe = v_safe; /* cap and proceed */

                    /* Reset accumulators, keep settle timer running */
                    fd_theta        = 0.0f;
                    fd_sample_count = 0;
                    fd_sum_sin      = 0.0f;
                    fd_sum_cos      = 0.0f;
                    fd_meas_count   = 0;
                    break; /* retry next ISR cycles */
                }

                if (I_amp_sq < 1e-6f) {
                    /* No current at max voltage — wiring issue */
                    id_result.error_code = 2;
                    id_result.state = MOTOR_ID_STATE_ERROR;
                    break;
                }

                /* Adequate current: compute phi with good SNR */
                float omega_fd = TWO_PI * ID_FD_PROBE_HZ;
                float rapp_fd  = fd_vprobe * I_re / I_amp_sq;
                float ls_fd    = fd_vprobe * I_im / (omega_fd * I_amp_sq);

                /* Impedance angle (ignoring small delay — acceptable for freq selection) */
                float phi_app = atan2f(fabsf(ls_fd * omega_fd), fabsf(rapp_fd));
                id_result.dbg_phi_detect_deg = phi_app * (180.0f / PI);

                /* Motor corner frequency f_corner = R/(2π·L):
                 * At f_probe: tan(φ) = ω·Ls/Rs → f_corner = f_probe / tan(φ) */
                float tan_phi   = tanf(phi_app);
                float f_corner  = (tan_phi > 0.01f) ? (ID_FD_PROBE_HZ / tan_phi) : 100000.0f;

                /* Target φ_m1 ≈ 25° for k=4 → f1 = f_corner × tan(25°) = f_corner × 0.4663 */
                float f1_ideal_k4 = f_corner * 0.4663f;
                float f1_k4       = find_nearest_candidate(f1_ideal_k4);
                float f2_k4       = f1_k4 * 4.0f;

                float chosen_f1, chosen_f2;
                if (f1_k4 >= 100.0f && f2_k4 <= 4000.0f && is_candidate(f2_k4) && f1_k4 != f2_k4) {
                    chosen_f1 = f1_k4;
                    chosen_f2 = f2_k4;
                } else {
                    /* Fallback: k=2, target φ_m1 ≈ 35° → f1 = f_corner × 0.7002 */
                    float f1_ideal_k2 = f_corner * 0.7002f;
                    float f1_k2       = find_nearest_candidate(f1_ideal_k2);
                    float f2_k2       = f1_k2 * 2.0f;

                    if (f1_k2 >= 100.0f && f2_k2 <= 4000.0f && is_candidate(f2_k2) && f1_k2 != f2_k2) {
                        chosen_f1 = f1_k2;
                        chosen_f2 = f2_k2;
                    } else {
                        /* Final fallback for very low L/R motors (drone BLDC):
                         * f_corner >> 4 kHz, use the two highest safe candidates. */
                        chosen_f1 = 2000.0f;
                        chosen_f2 = 4000.0f;
                    }
                }

                id_result.dbg_f1_hz = chosen_f1;
                id_result.dbg_f2_hz = chosen_f2;
                omega_f1 = TWO_PI * chosen_f1;
                omega_f2 = TWO_PI * chosen_f2;

                /* Enter MEASURE_LS_F1 */
                ls_update_freq_params(chosen_f1);
                ls_vprobe = ID_LS_PROBE_V_INIT;
                ls_vsweet = ID_LS_PROBE_V_INIT;
                ls_sub = 0; /* SETTLE */
                ls_reset_lockin();
                reset_timer();
                id_result.state = MOTOR_ID_STATE_MEASURE_LS_F1;
            }
        } break;

        /* ================================================================== */
        /* MEASURE_LS_F1 / MEASURE_LS_F2 — shared logic via ls_sub           */
        /*   Sub 0: SETTLE  (Vd=0, ID_LS_SETTLE_MS)                          */
        /*   Sub 1: PROBE   (sweep voltage to find Vsweet)                   */
        /*   Sub 2: MEASURE (full lock-in, store raw phasors)                */
        /* ================================================================== */
        case MOTOR_ID_STATE_MEASURE_LS_F1:
        case MOTOR_ID_STATE_MEASURE_LS_F2: {
            switch (ls_sub) {
                /* ---- SETTLE: output 0, wait for motor current to decay ---- */
                case 0:
                    *vd = 0.0f;
                    *vq = 0.0f;
                    if (id_timer_ms >= ID_LS_SETTLE_MS) {
                        ls_sub = 1;
                        reset_timer();
                    }
                    break;

                /* ---- PROBE: inject and check current amplitude ---- */
                case 1: {
                    ls_theta += ls_phase_inc;
                    if (ls_theta >= PI) ls_theta -= TWO_PI;

                    float s, c;
                    cordic_sincos(ls_theta / PI, &c, &s);
                    *vd = ls_vprobe * s;
                    *vq = 0.0f;

                    ls_sample_count++;

                    if (ls_sample_count > ls_skip_samp) {
                        ls_sum_sin += id * s;
                        ls_sum_cos += id * c;
                        ls_meas_count++;
                    }

                    if (ls_sample_count >= ls_probe_samp) {
                        float inv_N = 1.0f / (float)ls_meas_count;
                        float I_re = 2.0f * ls_sum_sin * inv_N;
                        float I_im = -2.0f * ls_sum_cos * inv_N;
                        float I_amp = sqrtf(I_re * I_re + I_im * I_im);

                        if (I_amp >= ID_LS_PROBE_I_MIN && I_amp <= ID_LS_PROBE_I_MAX) {
                            /* Good current range → set Vsweet, advance to MEASURE */
                            ls_vsweet = ls_vprobe;
                            ls_sub = 2;
                            ls_reset_lockin();
                            reset_timer();

                        } else if (I_amp > ID_LS_PROBE_I_MAX) {
                            /* Over-current: scale down */
                            ls_vsweet = ls_vprobe *
                                        ((ID_LS_PROBE_I_MIN + ID_LS_PROBE_I_MAX) * 0.5f / I_amp);
                            if (ls_vsweet < 0.5f) ls_vsweet = 0.5f;
                            ls_sub = 2;
                            ls_reset_lockin();
                            reset_timer();

                        } else {
                            /* Too little current: raise voltage */
                            ls_vprobe += ID_LS_PROBE_V_STEP;
                            float v_safe = 0.45f * vbus;
                            if (v_safe > ID_LS_PROBE_V_MAX) v_safe = ID_LS_PROBE_V_MAX;
                            if (ls_vprobe > v_safe) {
                                /* Max voltage reached but still low current.
                                 * Use V_safe as Vsweet (high-inductance motor). */
                                ls_vsweet = v_safe;
                                ls_sub = 2;
                                ls_reset_lockin();
                                reset_timer();
                            } else {
                                ls_reset_lockin();
                            }
                        }
                    }
                } break;

                /* ---- MEASURE: full lock-in, 25 cycles ---- */
                case 2: {
                    /* Settle time: output 0 until settled */
                    if (id_timer_ms < ID_LS_SETTLE_MS) {
                        *vd = 0.0f;
                        *vq = 0.0f;
                        break;
                    }

                    ls_theta += ls_phase_inc;
                    if (ls_theta >= PI) ls_theta -= TWO_PI;

                    float s, c;
                    cordic_sincos(ls_theta / PI, &c, &s);
                    *vd = ls_vsweet * s;
                    *vq = 0.0f;

                    ls_sample_count++;

                    if (ls_sample_count > ls_skip_samp) {
                        ls_sum_sin += id * s;
                        ls_sum_cos += id * c;
                        ls_meas_count++;
                    }

                    if (ls_sample_count >= ls_total_samp) {
                        float inv_N = 1.0f / (float)ls_meas_count;
                        float I_re = 2.0f * ls_sum_sin * inv_N;
                        float I_im = -2.0f * ls_sum_cos * inv_N;
                        float I_amp_sq = I_re * I_re + I_im * I_im;

                        if (I_amp_sq < 1e-4f) {
                            id_result.error_code = 2;
                            id_result.state = MOTOR_ID_STATE_ERROR;
                            break;
                        }

                        /* Store RAW phasors (NO delay compensation) */
                        float rapp_raw = ls_vsweet * I_re / I_amp_sq;
                        float ls_raw_v = ls_vsweet * I_im / (ls_omega * I_amp_sq);
                        float i_amp = sqrtf(I_amp_sq);

                        if (id_result.state == MOTOR_ID_STATE_MEASURE_LS_F1) {
                            rapp_raw_f1 = rapp_raw;
                            ls_raw_f1 = ls_raw_v;
                            iamp_f1 = i_amp;
                            id_result.dbg_rapp_raw_f1 = rapp_raw;
                            id_result.dbg_ls_raw_f1 = ls_raw_v;
                            id_result.dbg_iamp_f1 = i_amp;

                            /* Advance to MEASURE_LS_F2 */
                            ls_update_freq_params(id_result.dbg_f2_hz);
                            ls_vprobe = ID_LS_PROBE_V_INIT;
                            ls_vsweet = ID_LS_PROBE_V_INIT;
                            ls_sub = 0; /* Settle again before f2 */
                            ls_reset_lockin();
                            reset_timer();
                            id_result.state = MOTOR_ID_STATE_MEASURE_LS_F2;

                        } else { /* MEASURE_LS_F2 */
                            rapp_raw_f2 = rapp_raw;
                            ls_raw_f2 = ls_raw_v;
                            iamp_f2 = i_amp;
                            id_result.dbg_rapp_raw_f2 = rapp_raw;
                            id_result.dbg_ls_raw_f2 = ls_raw_v;
                            id_result.dbg_iamp_f2 = i_amp;

                            /* Enter EXTRACT */
                            ext_idx = 0;
                            ext_best_N = 0.0f;
                            ext_best_res = 1e9f;
                            ext_res_at_0 = fabsf(ls_raw_f1 - ls_raw_f2);
                            FOC_EnableDrivers(0);
                            id_result.state = MOTOR_ID_STATE_EXTRACT;
                        }
                    }
                } break;
            } /* switch ls_sub */
        } break;

        /* ================================================================== */
        /* EXTRACT: Scan N ∈ [0, 5] to minimise |Ls_c(f1,N) − Ls_c(f2,N)|  */
        /*          Spread across multiple ISR calls (ID_EXTRACT_PER_ISR pts) */
        /* ================================================================== */
        case MOTOR_ID_STATE_EXTRACT: {
            *vd = 0.0f;
            *vq = 0.0f;

            int end = ext_idx + ID_EXTRACT_PER_ISR;
            if (end > 100) end = 100;

            for (; ext_idx < end; ext_idx++) {
                float N = ID_EXTRACT_N_MIN + ext_idx * ID_EXTRACT_N_STEP;
                float phi1 = omega_f1 * N * CONTROL_PERIOD_F;
                float phi2 = omega_f2 * N * CONTROL_PERIOD_F;

                float cos1, sin1, cos2, sin2;
                cordic_sincos(phi1 / PI, &cos1, &sin1);
                cordic_sincos(phi2 / PI, &cos2, &sin2);

                /* Ls_c(f, N) = Ls_raw·cos(φ) − (Rapp_raw/ω)·sin(φ) */
                float Ls_c1 = ls_raw_f1 * cos1 - (rapp_raw_f1 / omega_f1) * sin1;
                float Ls_c2 = ls_raw_f2 * cos2 - (rapp_raw_f2 / omega_f2) * sin2;

                if (Ls_c1 > 0.0f && Ls_c2 > 0.0f) {
                    float residual = fabsf(Ls_c1 - Ls_c2);
                    if (residual < ext_best_res) {
                        ext_best_res = residual;
                        ext_best_N = N;
                    }
                }
            }

            if (ext_idx >= 100) {
                /* Scan complete — compute final results */

                /* Convergence check: minimum must be significantly better than N=0 residual.
                 * If ext_res_at_0 ≈ 0, both frequencies already agree (very high L/R motor):
                 * raw Ls already accurate, skip compensation. */
                float phi1_opt = omega_f1 * ext_best_N * CONTROL_PERIOD_F;
                float phi2_opt = omega_f2 * ext_best_N * CONTROL_PERIOD_F;

                float cos1, sin1, cos2, sin2;
                cordic_sincos(phi1_opt / PI, &cos1, &sin1);
                cordic_sincos(phi2_opt / PI, &cos2, &sin2);

                float Ls_c1 = ls_raw_f1 * cos1 - (rapp_raw_f1 / omega_f1) * sin1;
                float Ls_c2 = ls_raw_f2 * cos2 - (rapp_raw_f2 / omega_f2) * sin2;

                float Ls_final;
                float N_used;

                /* Converged if the minimum is at least 40% better than the N=0 residual */
                int converged = (ext_res_at_0 > 1e-9f) && (ext_best_res < 0.60f * ext_res_at_0);

                if (converged) {
                    Ls_final = (Ls_c1 + Ls_c2) * 0.5f;
                    N_used = ext_best_N;
                } else {
                    /* Flat residual → high L/R motor, delay error negligible.
                     * Use raw Ls at f1 with fixed 0.5-sample compensation. */
                    N_used = 0.5f;
                    float phi_fb = omega_f1 * N_used * CONTROL_PERIOD_F;
                    float cof, sif;
                    cordic_sincos(phi_fb / PI, &cof, &sif);
                    Ls_final = ls_raw_f1 * cof - (rapp_raw_f1 / omega_f1) * sif;

                    /* Recompute c1/c2 for debug fields */
                    phi1_opt = phi_fb;
                    phi2_opt = omega_f2 * N_used * CONTROL_PERIOD_F;
                    cordic_sincos(phi1_opt / PI, &cos1, &sin1);
                    cordic_sincos(phi2_opt / PI, &cos2, &sin2);
                    Ls_c1 = ls_raw_f1 * cos1 - (rapp_raw_f1 / omega_f1) * sin1;
                    Ls_c2 = ls_raw_f2 * cos2 - (rapp_raw_f2 / omega_f2) * sin2;
                }

                if (Ls_final < 1e-7f) Ls_final = 1e-7f; /* Clamp lower bound */

                id_result.measured_ls = Ls_final;
                id_result.identified_delay_samples = N_used;
                id_result.dbg_ls_comp_f1 = Ls_c1;
                id_result.dbg_ls_comp_f2 = Ls_c2;

                /* Dead-time estimation from compensated Rapp at f1:
                 *   Rapp1_comp = Rs + (4/π) * Vdead / Iamp1
                 *   → Vdead = (π/4) * (Rapp1_comp - Rs) * Iamp1 */
                float rapp1_comp = rapp_raw_f1 * cos1 + omega_f1 * ls_raw_f1 * sin1;
                float rs = id_result.measured_rs;

                if (rapp1_comp > rs && rs > 1e-4f && iamp_f1 > 0.1f) {
                    float v_err = (PI / 4.0f) * (rapp1_comp - rs) * iamp_f1;
                    float td_ns = (v_err / rs_vbus_avg) * (1e9f / (float)PWM_FREQUENCY);
                    id_result.identified_v_err = v_err;
                    id_result.identified_deadtime_ns = td_ns;
                } else {
                    id_result.identified_v_err = 0.0f;
                    id_result.identified_deadtime_ns = 0.0f;
                }

                id_result.state = MOTOR_ID_STATE_COMPLETE;
            }
        } break;

    } /* switch id_result.state */
}
