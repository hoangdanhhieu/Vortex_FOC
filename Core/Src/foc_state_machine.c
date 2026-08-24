/**
 * @file foc_state_machine.c
 * @brief FOC State Machine and Main Control Implementation
 */

#include "foc_state_machine.h"

#include "comm_protocol.h"
#include "cordic_math.h"
#include "foc.h"
#include "foc_config.h"
#include "main.h"
#include "math.h"
#include "motor_id.h"
#include "peripheral_init.h"

/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/

void FOC_StartSelfCommission(void);

/* Calibration samples count */
#define CAL_SAMPLES 512

/*===========================================================================*/
/* Global Instance                                                           */
/*===========================================================================*/

FOC_Control_t g_foc __attribute__((aligned(4))); /* aligned for efficient FPU access */
static volatile uint8_t foc_initialized = 0;

/* Safety check state */
static uint32_t stall_counter = 0;      /* Stall timer (ticks) */
static uint32_t ground_fault_count = 0; /* Ground fault deglitch counter */

/* Transition blending state (open-loop → closed-loop) */
static float s_blend_alpha = 0.0f; /* 0 = open-loop, 1 = closed-loop */
static uint32_t s_transition_counter = 0;
static uint32_t s_transition_samples = 0;
static uint8_t s_in_transition = 0;
static uint32_t s_closed_loop_counter = 0;

/* Stop ramp-down state */
static uint8_t s_stopping = 0;      /* 1 = ramp-down in progress */
static uint32_t s_stop_counter = 0; /* Timeout counter */
static float s_saved_speed_target = 0.0f;
static float s_saved_iq_target = 0.0f;

/* Flying start state */
static uint32_t s_detect_counter = 0;
static uint32_t s_detect_samples = 0;
static float s_detect_peak = 0.0f;
static float s_bemf_threshold = 0.0f;
static uint32_t s_flying_start_counter = 0;
static uint32_t s_lock_samples = 0;

static float s_E_alpha_prev = 0.0f;
static float s_E_beta_prev = 0.0f;
static float s_cross_prod_sum = 0.0f;

/* Phase voltage calibration & spinning detection accumulators */
static int32_t s_vphase_a_accum = 0;
static int32_t s_vphase_c_accum = 0;

static uint16_t s_vphase_a_min = 4096;
static uint16_t s_vphase_a_max = 0;
static uint16_t s_vphase_c_min = 4096;
static uint16_t s_vphase_c_max = 0;

/* Current sensing noise characterization accumulators */
static uint64_t s_adc1_sq_accum = 0;
static uint64_t s_adc2_b_sq_accum = 0;
static uint64_t s_adc2_c_sq_accum = 0;
static uint16_t s_adc1_min = 4096;
static uint16_t s_adc1_max = 0;
static uint16_t s_adc2_b_min = 4096;
static uint16_t s_adc2_b_max = 0;
static uint16_t s_adc2_c_min = 4096;
static uint16_t s_adc2_c_max = 0;

/**
 * @brief Tracks which phase each ADC was configured to sample.
 *        Set at END of each ISR for the NEXT trigger.
 *        0 = Phase A, 1 = Phase B, 2 = Phase C
 */
static uint8_t s_adc1_phase = 0; /* Default: ADC1 → Phase A (VOPAMP1) */
static uint8_t s_adc2_phase = 1; /* Default: ADC2 → Phase B (VOPAMP2) */
static uint8_t s_skip_phase = 2; /* Which phase is currently skipped (0=A, 1=B, 2=C) */

extern volatile uint32_t adc_isr_us;
extern volatile uint16_t adc_regular_buffer[2];
extern volatile float ADC_Vref;

/**
 * @brief Normalize angle to [-1, 1) range (corresponds to [-pi, pi))
 */
CCMRAM_FUNC static inline float normalize_angle_norm(float angle) {
    return angle - 2.0f * floorf((angle + 1.0f) * 0.5f);
}

/*===========================================================================*/
/* Private Functions                                                         */
/*===========================================================================*/

__attribute__((noinline)) static void FOC_StateIdle(void);
__attribute__((noinline)) static void FOC_StateCalibration(void);
__attribute__((noinline)) static void FOC_StateDetect(void);
__attribute__((noinline)) static void FOC_StateFlyingStart(void);
__attribute__((noinline)) static void FOC_StateAlign(void);
__attribute__((noinline)) static void FOC_StateStartup(void);
static void FOC_StateRun(void);
__attribute__((noinline)) static void FOC_StateStop(void);
__attribute__((noinline)) static void FOC_StateFault(void);
__attribute__((noinline)) static void FOC_StateSelfCommission(void);
static inline void FOC_ApplyDeadtimeCompensation(float* out_a, float* out_b, float* out_c);

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

void FOC_Init(void) {
    foc_initialized = 0;

    g_foc.status.state = FOC_STATE_IDLE;
    g_foc.status.control_mode = FOC_MODE_SPEED;
    g_foc.status.fault = FOC_FAULT_NONE;

    g_foc.data.Ia = g_foc.data.Ib = g_foc.data.Ic = 0.0f;
    g_foc.data.Ialpha = g_foc.data.Ibeta = 0.0f;
    g_foc.data.Ialpha_flt = g_foc.data.Ibeta_flt = 0.0f;
    g_foc.data.Id = g_foc.data.Iq = 0.0f;
    g_foc.data.Vd = g_foc.data.Vq = 0.0f;
    g_foc.data.Iq_ref_cmd = 0.0f;
    g_foc.data.Valpha = g_foc.data.Vbeta = 0.0f;
    g_foc.data.Vbus = 12.0f;

    g_foc.cmd.Id_ref = 0.0f;
    g_foc.cmd.Iq_ref = 0.0f;
    g_foc.cmd.Vq_ref = 0.0f;
    g_foc.cmd.Vq_ref_target = 0.0f;
    g_foc.cmd.speed_ref = 0.0f;
    g_foc.data.theta_elec = 0.0f;
    g_foc.data.omega_elec = 0.0f;
    g_foc.data.speed_rpm = 0.0f;
    g_foc.status.in_transition = 0;

    g_foc.startup.theta = 0.0f;
    g_foc.startup.omega = 0.0f;
    g_foc.startup.counter = 0;

    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    PI_Init(&g_foc.ctrl.id, PI_ID_KP, PI_ID_KI, -12.0f, 12.0f, CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.id, -12.0f, 12.0f);

    PI_Init(&g_foc.ctrl.iq, PI_IQ_KP, PI_IQ_KI, -12.0f, 12.0f, CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.iq, -12.0f, 12.0f);

    LADRC_Init(&g_foc.ctrl.speed, LADRC_OMEGA_C_DEFAULT, LADRC_OMEGA_O_DEFAULT, LADRC_B0_DEFAULT,
               PI_SPEED_OUT_MIN, PI_SPEED_OUT_MAX, 0.001f);

    SMO_Init(&g_foc.ctrl.smo);

    BIST_Init(&g_foc.ctrl.bist);

    g_foc.adc_cal.offset_a = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_b = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_c = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_vphase_a = 234;
    g_foc.adc_cal.offset_vphase_b = 234;
    g_foc.adc_cal.offset_vphase_c = 234;
    g_foc.adc_cal.cal_samples = 0;

    g_foc.status.run_counter = 0;
    g_foc.isr_time_cycles = 0;

    FOC_EnableDrivers(0);

    MotorID_Init();

    g_foc.status.reverse = 1.0f;

    g_foc.cfg.startup_current = STARTUP_CURRENT;
    g_foc.cfg.align_current = ALIGN_CURRENT;
    g_foc.cfg.startup_accel = STARTUP_ACCEL;
    g_foc.cfg.startup_handoff_speed = STARTUP_HANDOFF_SPEED;
    g_foc.cfg.speed_ramp_accel = SPEED_RAMP_ACCEL;
    g_foc.cfg.speed_ramp_decel = SPEED_RAMP_DECEL;
    g_foc.cfg.ladrc_omega_c = LADRC_OMEGA_C_DEFAULT;
    g_foc.cfg.ladrc_omega_o = LADRC_OMEGA_O_DEFAULT;
    g_foc.cfg.ladrc_b0 = LADRC_B0_DEFAULT;
    g_foc.cfg.motor_inertia = MOTOR_INERTIA;
    g_foc.cfg.fault_oc_threshold = FAULT_OVERCURRENT_THRESHOLD;
    g_foc.cfg.fault_oc_count = (uint8_t)FAULT_OVERCURRENT_COUNT;
    g_foc.cfg.fault_ov_threshold = FAULT_OVERVOLTAGE_THRESHOLD;
    g_foc.cfg.fault_uv_threshold = FAULT_UNDERVOLTAGE_THRESHOLD;
    g_foc.cfg.fault_stall_enable = (uint8_t)FAULT_STALL_ENABLE;
    g_foc.cfg.fault_stall_speed = FAULT_STALL_SPEED_RPM;
    g_foc.cfg.fault_stall_current = FAULT_STALL_CURRENT_A;
    g_foc.cfg.fault_stall_time_ms = FAULT_STALL_TIME_MS;

    foc_initialized = 1;
}

void FOC_Start(void) {
    if (g_foc.status.state == FOC_STATE_IDLE) {
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        LADRC_Reset(&g_foc.ctrl.speed);
        SMO_Reset(&g_foc.ctrl.smo);

        /* Ensure PI controllers use original default gains from config */
        PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
        PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);

        s_blend_alpha = 0.0f;
        s_transition_counter = 0;
        s_in_transition = 0;
        g_foc.status.in_transition = 0;

        g_foc.data.Iq_ref_cmd = 0.0f;

        s_stopping = 0;
        s_stop_counter = 0;

        float f_elec_min = g_foc.cfg.motor_min_spd * g_foc.cfg.motor_poles / 60.0f;
        if (f_elec_min < 1.0f) f_elec_min = 1.0f;
        s_detect_samples = (uint32_t)(2.0f / f_elec_min * (float)CONTROL_FREQUENCY);
        s_lock_samples = (uint32_t)(5.0f / (TWO_PI * SMO_PPL_CUTOFF) * (float)CONTROL_FREQUENCY);
        uint32_t min_lock_samples = (uint32_t)(0.05f * (float)CONTROL_FREQUENCY);  // 50 ms
        if (s_lock_samples < min_lock_samples) s_lock_samples = min_lock_samples;
        s_bemf_threshold = g_foc.cfg.motor_min_spd / g_foc.cfg.motor_kv * 0.8f;
        if (s_bemf_threshold < 0.05f) s_bemf_threshold = 0.05f;

        s_detect_counter = 0;
        s_detect_peak = 0.0f;
        s_flying_start_counter = 0;

        s_E_alpha_prev = 0.0f;
        s_E_beta_prev = 0.0f;
        s_cross_prod_sum = 0.0f;

        g_foc.adc_cal.offset_a = 0;
        g_foc.adc_cal.offset_b = 0;
        g_foc.adc_cal.offset_c = 0;
        g_foc.adc_cal.cal_samples = 0;

        s_vphase_a_accum = 0;
        s_vphase_c_accum = 0;
        s_vphase_a_min = 4096;
        s_vphase_a_max = 0;
        s_vphase_c_min = 4096;
        s_vphase_c_max = 0;

        s_adc1_sq_accum = 0;
        s_adc2_b_sq_accum = 0;
        s_adc2_c_sq_accum = 0;
        s_adc1_min = 4096;
        s_adc1_max = 0;
        s_adc2_b_min = 4096;
        s_adc2_b_max = 0;
        s_adc2_c_min = 4096;
        s_adc2_c_max = 0;

        /* Reset ADC channel switching to default (skip C) */
        s_skip_phase = 2;
        s_adc1_phase = 0;
        s_adc2_phase = 1;
        LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
        LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
        LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);

        FOC_SetPhaseVoltageDMA(1);
        g_foc.status.state = FOC_STATE_CALIBRATION;

        float v_limit = g_foc.data.Vbus * SQRT3_INV;
        PI_SetLimits(&g_foc.ctrl.id, -v_limit, v_limit);
        PI_SetIntLimits(&g_foc.ctrl.id, -v_limit, v_limit);
        PI_SetLimits(&g_foc.ctrl.iq, -v_limit, v_limit);
        PI_SetIntLimits(&g_foc.ctrl.iq, -v_limit, v_limit);
    }
}

void FOC_Stop(void) {
    if (g_foc.status.state == FOC_STATE_STARTUP || 1) {
        s_stopping = 0;
    } else if (g_foc.status.state == FOC_STATE_RUN) {
        s_saved_speed_target = g_foc.cmd.speed_ref_target;
        s_saved_iq_target = g_foc.cmd.Iq_ref_target;
        g_foc.cmd.speed_ref_target = 0.0f;
        g_foc.cmd.Iq_ref_target = 0.0f;
        g_foc.cmd.Vq_ref_target = 0.0f;
        s_stopping = 1;
        s_stop_counter = 0;
    }
    g_foc.status.state = FOC_STATE_STOP;
}

void FOC_SlowTask(void) {
    if (!foc_initialized) return;

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

            /* Rate-limit current command using current_ramp_rate for bumpless torque delivery */
            float current_ramp_step = g_foc.cfg.current_ramp_rate * 0.001f;
            float iq_ramp_err = target_iq - g_foc.cmd.Iq_ref;
            if (iq_ramp_err > current_ramp_step) {
                g_foc.cmd.Iq_ref += current_ramp_step;
            } else if (iq_ramp_err < -current_ramp_step) {
                g_foc.cmd.Iq_ref -= current_ramp_step;
            } else {
                g_foc.cmd.Iq_ref = target_iq;
            }

            /* Anti-windup tracking for LESO */
            g_foc.ctrl.speed.u_prev = g_foc.cmd.Iq_ref;
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

/**
 * @brief Apply centralized deadtime compensation to duty cycles.
 *        To prevent self-excited chattering/oscillations during open-loop states (e.g.
 * startup/alignment) due to current measurement noise, we use command/reference current vector
 * (Id_ref, Iq_ref) and electrical angle (theta_elec) to determine the signs of currents. During
 * parameter identification, we fall back to IIR low-pass filtered measured currents.
 */
CCMRAM_FUNC static inline void FOC_ApplyDeadtimeCompensation(float* out_a, float* out_b,
                                                             float* out_c) {
    float dt_comp = DEAD_TIME_DUTY;
    if (dt_comp <= 0.0f) {
        return;
    }

    float sign_a, sign_b, sign_c;
    float i_th;

    if (g_foc.status.state == FOC_STATE_SELF_COMMISSION) {
        return;
    } else if (g_foc.status.state == FOC_STATE_RUN || g_foc.status.state == FOC_STATE_STOP) {
        // sign_a = g_foc.data.Ia;
        // sign_b = g_foc.data.Ib;
        // sign_c = g_foc.data.Ic;
        // i_th = 2.0f;
        return;
    } else {
        float sin_th, cos_th;
        cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);

        float I_alpha_ref = g_foc.cmd.Id_ref * cos_th - g_foc.cmd.Iq_ref * sin_th;
        float I_beta_ref = g_foc.cmd.Id_ref * sin_th + g_foc.cmd.Iq_ref * cos_th;

        sign_a = I_alpha_ref;
        sign_b = -0.5f * I_alpha_ref + 0.8660254f * I_beta_ref;
        sign_c = -0.5f * I_alpha_ref - 0.8660254f * I_beta_ref;

        i_th = 0.05f;
    }

    if (sign_a > i_th)
        *out_a += dt_comp;
    else if (sign_a < -i_th)
        *out_a -= dt_comp;
    else
        *out_a += dt_comp * (sign_a / i_th);

    if (sign_b > i_th)
        *out_b += dt_comp;
    else if (sign_b < -i_th)
        *out_b -= dt_comp;
    else
        *out_b += dt_comp * (sign_b / i_th);

    if (sign_c > i_th)
        *out_c += dt_comp;
    else if (sign_c < -i_th)
        *out_c -= dt_comp;
    else
        *out_c += dt_comp * (sign_c / i_th);

    float max_duty = g_foc.max_duty;
    if (*out_a < 0.0f)
        *out_a = 0.0f;
    else if (*out_a > max_duty)
        *out_a = max_duty;
    if (*out_b < 0.0f)
        *out_b = 0.0f;
    else if (*out_b > max_duty)
        *out_b = max_duty;
    if (*out_c < 0.0f)
        *out_c = 0.0f;
    else if (*out_c > max_duty)
        *out_c = max_duty;
}

/*===========================================================================*/
/* Main ISR Entry Point                                                      */
/*===========================================================================*/
CCMRAM_FUNC void FOC_HighFrequencyTask(uint16_t adc1_data, uint16_t adc2_data) {
    if (!foc_initialized) return;
    if (g_foc.status.state != FOC_STATE_RUN) {
        s_closed_loop_counter = 0;
        g_foc.ctrl.smo.enable_harmonic_comp = 0;
    }

    float i_scale = g_foc.data.i_scale;
    if (i_scale == 0.0f) {
        float vref = ADC_Vref;
        if (vref < 1.0f) vref = 3.3f;
        i_scale = vref * (ADC_TO_CURRENT);
    }

    /* Demux ADC data based on which phase was configured last ISR */
    uint16_t adc_ia = 0, adc_ib = 0, adc_ic = 0;

    switch (s_adc1_phase) {
        case 0:
            adc_ia = adc1_data;
            break; /* ADC1 measured Phase A */
        case 2:
            adc_ic = adc1_data;
            break; /* ADC1 measured Phase C (PB1) */
        default:
            break;
    }
    switch (s_adc2_phase) {
        case 1:
            adc_ib = adc2_data;
            break; /* ADC2 measured Phase B */
        case 2:
            adc_ic = adc2_data;
            break; /* ADC2 measured Phase C (VOPAMP3) */
        default:
            break;
    }

    /* Convert measured phases to current, reconstruct missing via Kirchhoff */
    if (s_skip_phase != 0)
        g_foc.data.Ia = ((float)adc_ia - (float)g_foc.adc_cal.offset_a) * i_scale;
    if (s_skip_phase != 1)
        g_foc.data.Ib = ((float)adc_ib - (float)g_foc.adc_cal.offset_b) * i_scale;
    if (s_skip_phase != 2)
        g_foc.data.Ic = ((float)adc_ic - (float)g_foc.adc_cal.offset_c) * i_scale;

    switch (s_skip_phase) {
        case 0:
            g_foc.data.Ia = -(g_foc.data.Ib + g_foc.data.Ic);
            break;
        case 1:
            g_foc.data.Ib = -(g_foc.data.Ia + g_foc.data.Ic);
            break;
        default:
            g_foc.data.Ic = -(g_foc.data.Ia + g_foc.data.Ib);
            break;
    }
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_FAULT) {
        if (g_foc.status.state == FOC_STATE_IDLE) {
            FOC_StateIdle();
        } else {
            FOC_StateFault();
        }

        /* Plot streaming: snapshot current data every N-th ISR cycle */
        static uint16_t plot_div = 0;
        if (g_foc.plot.enabled && ++plot_div >= PLOT_DECIMATION) {
            plot_div = 0;
            Comm_PushPlotSample7(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Ic,
                                 g_foc.data.Vd, g_foc.data.Vq, g_foc.data.theta_elec,
                                 g_foc.cmd.Iq_ref);
        }

        foc_set_pwm_duty(0.5f, 0.5f, 0.5f);
        g_foc.status.run_counter++;
        return;
    }
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, adc1_data);
    g_foc.status.run_counter++;

    if (g_foc.status.reverse < 0.0f) {
        float t = g_foc.data.Ib;
        g_foc.data.Ib = g_foc.data.Ic;
        g_foc.data.Ic = t;
    }

    clarke_transform(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Ic, &g_foc.data.Ialpha,
                     &g_foc.data.Ibeta);

    float wc = TWO_PI * CURRENT_STF_FC;
    float a = wc * CONTROL_PERIOD;
    float b = g_foc.data.omega_elec * CONTROL_PERIOD;
    float D_inv = 1.0f / ((1.0f + a) * (1.0f + a) + b * b);
    float ri_alpha = g_foc.data.Ialpha_flt + a * g_foc.data.Ialpha;
    float ri_beta = g_foc.data.Ibeta_flt + a * g_foc.data.Ibeta;
    g_foc.data.Ialpha_flt = ((1.0f + a) * ri_alpha - b * ri_beta) * D_inv;
    g_foc.data.Ibeta_flt = (b * ri_alpha + (1.0f + a) * ri_beta) * D_inv;

    if (g_foc.status.state == FOC_STATE_DETECT || g_foc.status.state == FOC_STATE_FLYING_START) {
        g_foc.data.Vphase_a =
            foc_adc_to_vphase(adc_regular_buffer[0], g_foc.adc_cal.offset_vphase_a);
        g_foc.data.Vphase_c =
            foc_adc_to_vphase(adc_regular_buffer[1], g_foc.adc_cal.offset_vphase_c);
        /* Reconstruct Vb from 2-phase: Vb = -(Va + Vc) */
        g_foc.data.Vphase_b = -(g_foc.data.Vphase_a + g_foc.data.Vphase_c);
    }

    switch (g_foc.status.state) {
        case FOC_STATE_IDLE:
            FOC_StateIdle();
            break;
        case FOC_STATE_CALIBRATION:
            /* ADC1 always measures Phase A */
            g_foc.adc_cal.offset_a += adc1_data;
            s_adc1_sq_accum += (uint64_t)adc1_data * adc1_data;
            if (adc1_data < s_adc1_min) s_adc1_min = adc1_data;
            if (adc1_data > s_adc1_max) s_adc1_max = adc1_data;

            /* ADC2 alternates: even=Phase B (VOPAMP2), odd=Phase C (VOPAMP3) */
            if ((g_foc.adc_cal.cal_samples & 1) == 0) {
                g_foc.adc_cal.offset_b += adc2_data;
                s_adc2_b_sq_accum += (uint64_t)adc2_data * adc2_data;
                if (adc2_data < s_adc2_b_min) s_adc2_b_min = adc2_data;
                if (adc2_data > s_adc2_b_max) s_adc2_b_max = adc2_data;
            } else {
                g_foc.adc_cal.offset_c += adc2_data;
                s_adc2_c_sq_accum += (uint64_t)adc2_data * adc2_data;
                if (adc2_data < s_adc2_c_min) s_adc2_c_min = adc2_data;
                if (adc2_data > s_adc2_c_max) s_adc2_c_max = adc2_data;
            }
            {
                uint16_t va = adc_regular_buffer[0];
                uint16_t vc = adc_regular_buffer[1];
                s_vphase_a_accum += va;
                s_vphase_c_accum += vc;
                if (va < s_vphase_a_min) s_vphase_a_min = va;
                if (va > s_vphase_a_max) s_vphase_a_max = va;
                if (vc < s_vphase_c_min) s_vphase_c_min = vc;
                if (vc > s_vphase_c_max) s_vphase_c_max = vc;
            }
            g_foc.adc_cal.cal_samples++;
            FOC_StateCalibration();
            break;
        case FOC_STATE_DETECT:
            FOC_StateDetect();
            break;
        case FOC_STATE_FLYING_START:
            FOC_StateFlyingStart();
            break;
        case FOC_STATE_ALIGN:
            FOC_StateAlign();
            break;
        case FOC_STATE_STARTUP:
        case FOC_STATE_RUN:
#if DEBUG_RUN_TIMEOUT_MS > 0
            if (g_foc.status.run_counter >= (DEBUG_RUN_TIMEOUT_MS * CONTROL_FREQUENCY / 1000)) {
                g_foc.status.state = FOC_STATE_STOP;
                break;
            }
#endif
            if (g_foc.status.state == FOC_STATE_STARTUP) {
                FOC_StateStartup();
            } else {
                FOC_StateRun();
            }
            break;

        case FOC_STATE_STOP:
            if (s_stopping) {
                FOC_StateRun();
            } else {
                FOC_StateStop();
            }
            break;

        case FOC_STATE_FAULT:
            FOC_StateFault();
            break;

        case FOC_STATE_SELF_COMMISSION:
            FOC_StateSelfCommission();
            break;
    }

    /* Plot streaming: snapshot current data every N-th ISR cycle */
    {
        static uint16_t plot_div = 0;
        if (g_foc.plot.enabled && ++plot_div >= PLOT_DECIMATION) {
            plot_div = 0;
            Comm_PushPlotSample7(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Ic,
                                 g_foc.data.Vd, g_foc.data.Vq, g_foc.data.theta_elec,
                                 g_foc.cmd.Iq_ref);
        }
    }

    /*=======================================================================*/
    /* ADC Channel Switching for NEXT trigger (duty-based + hysteresis)      */
    /*=======================================================================*/
    if (g_foc.status.state != FOC_STATE_CALIBRATION) {
        float duties[3] = {g_foc.data.duty_a, g_foc.data.duty_b, g_foc.data.duty_c};
        uint8_t new_skip = s_skip_phase; /* Default: keep current skip */

        /* Only switch if a different phase clearly exceeds the current skip phase */
        for (uint8_t i = 0; i < 3; i++) {
            if (i != s_skip_phase && duties[i] > duties[s_skip_phase] + SKIP_HYSTERESIS) {
                new_skip = i;
            }
        }
        s_skip_phase = new_skip;

        /* Apply channel configuration based on skip decision */
        switch (s_skip_phase) {
            case 0: /* Skip Phase A → measure C(ADC1 via PB1) + B(ADC2) */
                LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_12);
                LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
                LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
                s_adc1_phase = 2;
                s_adc2_phase = 1;
                break;
            case 1: /* Skip Phase B → measure A(ADC1) + C(ADC2 via VOPAMP3) */
                LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
                LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP3_ADC2);
                LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
                s_adc1_phase = 0;
                s_adc2_phase = 2;
                break;
            default: /* Skip Phase C → measure A(ADC1) + B(ADC2) */
                LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
                LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
                LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
                s_adc1_phase = 0;
                s_adc2_phase = 1;
                break;
        }
    } else {
        /* During calibration: alternate ADC2 between Phase B and Phase C */
        if ((g_foc.adc_cal.cal_samples & 1) == 0) {
            /* Next ISR (odd) will measure Phase C */
            LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP3_ADC2);
        } else {
            /* Next ISR (even) will measure Phase B */
            LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
        }
    }

    float out_a = g_foc.data.duty_a;
    float out_b = g_foc.data.duty_b;
    float out_c = g_foc.data.duty_c;

    // FOC_ApplyDeadtimeCompensation(&out_a, &out_b, &out_c);

    if (g_foc.status.reverse > 0) {
        foc_set_pwm_duty(out_a, out_b, out_c);
    } else {
        foc_set_pwm_duty(out_a, out_c, out_b);
    }
}

/*===========================================================================*/
/* State Implementations                                                     */
/*===========================================================================*/

static void FOC_StateIdle(void) {
    // playTune();
    FOC_EnableDriver(1, 0);
    FOC_EnableDriver(2, 0);
    FOC_EnableDriver(3, 0);

    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;

    g_foc.data.Ialpha_flt = 0.0f;
    g_foc.data.Ibeta_flt = 0.0f;
}

void FOC_ConfigureAWD(void) {
    if (g_foc.adc_cal.offset_a < 200 || g_foc.adc_cal.offset_b < 200 ||
        g_foc.adc_cal.offset_c < 200) {
        LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, 4095, 0);
        LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD2, 4095, 0);
        LL_ADC_ConfigAnalogWDThresholds(ADC2, LL_ADC_AWD1, 4095, 0);

        LL_ADC_DisableIT_AWD1(ADC1);
        LL_ADC_DisableIT_AWD2(ADC1);
        LL_ADC_DisableIT_AWD1(ADC2);
        return;
    }

    float vref = ADC_Vref;
    if (vref < 1.0f) vref = 3.3f;

    // 1. Calculate Overcurrent thresholds in LSB counts
    const float gain = ADC_RESOLUTION * OPAMP_GAIN * SHUNT_RESISTANCE;
    float current_to_adc = gain / vref;
    float adc_step = g_foc.cfg.fault_oc_threshold * current_to_adc;

    // Phase A (ADC1 AWD1)
    int32_t high_a = g_foc.adc_cal.offset_a + (int32_t)adc_step;
    int32_t low_a = g_foc.adc_cal.offset_a - (int32_t)adc_step;
    if (high_a > 4095) high_a = 4095;
    if (low_a < 0) low_a = 0;

    // Phase B and C (ADC2 AWD1 - monitors all injected channels)
    int32_t avg_offset_bc = (g_foc.adc_cal.offset_b + g_foc.adc_cal.offset_c) / 2;
    int32_t high_bc = avg_offset_bc + (int32_t)adc_step;
    int32_t low_bc = avg_offset_bc - (int32_t)adc_step;
    if (high_bc > 4095) high_bc = 4095;
    if (low_bc < 0) low_bc = 0;

    // 2. Vbus protection is now handled in software (FOC_SlowTask at 1kHz)

    // 3. Program hardware registers using LL driver
    // ADC1: AWD1 for Phase A overcurrent
    LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD1, LL_ADC_AWD_CH_VOPAMP1_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, high_a, low_a);
    LL_ADC_SetAWDFilteringConfiguration(ADC1, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);

    // ADC2: AWD1 for all Injected Channels (Phase B and Phase C)
    LL_ADC_SetAnalogWDMonitChannels(ADC2, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC2, LL_ADC_AWD1, high_bc, low_bc);
    LL_ADC_SetAWDFilteringConfiguration(ADC2, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);

    // Clear all pending AWD flags first to prevent stale interrupt triggers
    LL_ADC_ClearFlag_AWD1(ADC1);
    LL_ADC_ClearFlag_AWD1(ADC2);

    // Now safely enable the AWD interrupts
    LL_ADC_EnableIT_AWD1(ADC1);
    LL_ADC_EnableIT_AWD1(ADC2);
}

static void FOC_StateCalibration(void) {
    if (g_foc.adc_cal.cal_samples >= CAL_SAMPLES) {
        float n_a = (float)CAL_SAMPLES;
        float n_bc = (float)(CAL_SAMPLES / 2.0);

        float mean_a = (float)g_foc.adc_cal.offset_a / n_a;
        float mean_b = (float)g_foc.adc_cal.offset_b / n_bc;
        float mean_c = (float)g_foc.adc_cal.offset_c / n_bc;

        g_foc.adc_cal.offset_a = (int32_t)(mean_a + 0.5f);
        g_foc.adc_cal.offset_b = (int32_t)(mean_b + 0.5f);
        g_foc.adc_cal.offset_c = (int32_t)(mean_c + 0.5f);

        /* Calculate Current Sensing RMS Noise & Peak-to-Peak Noise */
        float var_a = ((float)s_adc1_sq_accum / n_a) - (mean_a * mean_a);
        float var_b = ((float)s_adc2_b_sq_accum / n_bc) - (mean_b * mean_b);
        float var_c = ((float)s_adc2_c_sq_accum / n_bc) - (mean_c * mean_c);
        if (var_a < 0.0f) var_a = 0.0f;
        if (var_b < 0.0f) var_b = 0.0f;
        if (var_c < 0.0f) var_c = 0.0f;

        float i_scale = ADC_Vref * fabsf(ADC_TO_CURRENT);
        float noise_rms_a = sqrtf(var_a) * i_scale;
        float noise_rms_b = sqrtf(var_b) * i_scale;
        float noise_rms_c = sqrtf(var_c) * i_scale;
        float noise_rms = (noise_rms_a + noise_rms_b + noise_rms_c) * (1.0f / 3.0f);

        float pk_pk_a = (float)(s_adc1_max - s_adc1_min) * i_scale;
        float pk_pk_b = (float)(s_adc2_b_max - s_adc2_b_min) * i_scale;
        float pk_pk_c = (float)(s_adc2_c_max - s_adc2_c_min) * i_scale;
        float noise_pk_pk = pk_pk_a > pk_pk_b ? pk_pk_a : pk_pk_b;
        if (pk_pk_c > noise_pk_pk) noise_pk_pk = pk_pk_c;

        g_foc.noise_profile.noise_rms = noise_rms;
        g_foc.noise_profile.noise_pk_pk = noise_pk_pk;
        g_foc.noise_profile.is_flat_thr = clampf(1.5f * noise_pk_pk, 0.003f, 0.100f);
        g_foc.noise_profile.i_inj_min = clampf(10.0f * noise_rms, 0.050f, 0.500f);
        g_foc.noise_profile.bemf_noise_sq = 1e-4f;

        /* Hardware Health Evaluation */
        if (noise_rms < 0.015f) {
            g_foc.noise_profile.health_status = 0; /* EXCELLENT (< 15mA) */
        } else if (noise_rms < 0.040f) {
            g_foc.noise_profile.health_status = 1; /* GOOD (15 - 40mA) */
        } else if (noise_rms < 0.100f) {
            g_foc.noise_profile.health_status = 2; /* NOISY (40 - 100mA) */
        } else {
            g_foc.noise_profile.health_status = 3; /* FAULT (> 100mA) */
        }

        /* Restore ADC2 to default (VOPAMP2) after calibration */
        LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
        s_adc1_phase = 0;
        s_adc2_phase = 1;
        s_skip_phase = 2;

        /* Check if peak-to-peak variation on any phase is above threshold (30 ADC counts)
         * to detect if the motor is already spinning. */
        uint16_t diff_a = s_vphase_a_max - s_vphase_a_min;
        uint16_t diff_c = s_vphase_c_max - s_vphase_c_min;

        uint8_t is_spinning = (diff_a > 30) || (diff_c > 30);
        if (!is_spinning) {
            g_foc.adc_cal.offset_vphase_a = s_vphase_a_accum / CAL_SAMPLES;
            g_foc.adc_cal.offset_vphase_c = s_vphase_c_accum / CAL_SAMPLES;
        }

        // Configure hardware watchdogs dynamically based on offsets
        FOC_ConfigureAWD();

        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        g_foc.startup.counter = 0;

        if (id_result.state != MOTOR_ID_STATE_ALIGN) {
            g_foc.status.state = FOC_STATE_DETECT;
        } else {
            FOC_SetPhaseVoltageDMA(0);
            FOC_EnableDrivers(1);
            g_foc.status.state = FOC_STATE_SELF_COMMISSION;
        }
    }
}

static void FOC_StateDetect(void) {
    s_detect_counter++;
    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    float Ea = g_foc.data.Vphase_a;
    float Eb = g_foc.data.Vphase_b;
    float Ec = g_foc.data.Vphase_c;
    float E_alpha = Ea - 0.5f * Eb - 0.5f * Ec;
    float E_beta = 0.866f * (Eb - Ec);
    float amp = E_alpha * E_alpha + E_beta * E_beta;

    /* Low-Pass Filter on amplitude squared (DC value) to reject noise spikes */
    s_detect_peak += 0.05f * (amp - s_detect_peak);

    /* Cross-product for direction detection */
    float cross = s_E_alpha_prev * E_beta - s_E_beta_prev * E_alpha;
    s_cross_prod_sum += cross;
    s_E_alpha_prev = E_alpha;
    s_E_beta_prev = E_beta;

    if (s_detect_counter >= s_detect_samples) {
        float thr_sq = s_bemf_threshold * s_bemf_threshold;
        if (s_detect_peak > thr_sq) {
            SMO_Reset(&g_foc.ctrl.smo);

            float E_amp_real = sqrtf(s_detect_peak) * TWO_THIRDS;
            float est_omega = E_amp_real / g_foc.cfg.motor_flux;

            /* Apply detected direction */
            float sign_w = (s_cross_prod_sum >= 0.0f) ? 1.0f : -1.0f;
            est_omega *= sign_w;

            if (fabsf(est_omega) > g_foc.ctrl.smo.pll_int_max) {
                est_omega =
                    (est_omega > 0.0f) ? g_foc.ctrl.smo.pll_int_max : -g_foc.ctrl.smo.pll_int_max;
            }

            float theta_init = cordic_atan2(-E_alpha * sign_w, E_beta * sign_w);
            g_foc.ctrl.smo.theta_est = theta_init;

            g_foc.ctrl.smo.omega_est = est_omega;
            g_foc.ctrl.smo.omega_out = est_omega;
            g_foc.ctrl.smo.pll_integral = est_omega;
            g_foc.data.omega_elec = est_omega;
            g_foc.data.speed_rpm = (est_omega / g_foc.cfg.motor_poles) * (60.0f / TWO_PI);

            s_flying_start_counter = 0;
            g_foc.status.state = FOC_STATE_FLYING_START;
        } else {
            FOC_SetPhaseVoltageDMA(0);
            FOC_EnableDrivers(1);
            g_foc.status.state = FOC_STATE_ALIGN;
        }
    }
}

static void FOC_StateFlyingStart(void) {
    s_flying_start_counter++;

    float Ea = g_foc.data.Vphase_a;
    float Eb = g_foc.data.Vphase_b;
    float Ec = g_foc.data.Vphase_c;
    float E_alpha = Ea - 0.5f * Eb - 0.5f * Ec;
    float E_beta = 0.866f * (Eb - Ec);

    SMO_FeedBEMF(&g_foc.ctrl.smo, E_alpha, E_beta);

    float omega_now = g_foc.ctrl.smo.omega_est;
    g_foc.data.omega_elec = omega_now;
    g_foc.data.speed_rpm = (omega_now / g_foc.cfg.motor_poles) * (60.0f / TWO_PI);

    if (s_flying_start_counter < s_lock_samples) {
        g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    } else if (s_flying_start_counter == s_lock_samples) {
        float theta_now = SMO_GetAngle(&g_foc.ctrl.smo);
        float E_bemf = omega_now * g_foc.cfg.motor_flux;

        g_foc.data.Vd = 0.0f;
        g_foc.data.Vq = E_bemf;

        svpwm_calculate(theta_now);

    } else {
        /* -----------------------------------------------------------
         * ENABLE CYCLE: The timer update event between the previous
         * ISR and this one has loaded the correct duty from the
         * precharge cycle into the CCR shadow register. The PWM output
         * NOW matches back-EMF. Safe to enable gate drivers — the
         * very first PWM pulse the motor sees is correct.
         * ----------------------------------------------------------- */
        float theta_now = SMO_GetAngle(&g_foc.ctrl.smo);
        float E_bemf = omega_now * g_foc.cfg.motor_flux;

        /* Recompute SVPWM for the updated angle (one cycle later) */
        g_foc.data.Vd = 0.0f;
        g_foc.data.Vq = E_bemf;

        float sin_th, cos_th;
        cordic_sincos(theta_now, &cos_th, &sin_th);

        svpwm_calculate(theta_now);

        /* Pre-load current PI integrals for bumpless transfer (compensate for 0.5*E_bemf
         * feedforward in RUN) */
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        LADRC_Reset(&g_foc.ctrl.speed);
        g_foc.ctrl.iq.integral =
            0.5f * E_bemf; /* Vq starts at back-EMF level (0.5 PI integral + 0.5 FF = 1.0 E_bemf) */

        /* Seed SMO current observer with actual measured currents */
        g_foc.ctrl.smo.Ialpha_est = g_foc.data.Ialpha;
        g_foc.ctrl.smo.Ibeta_est = g_foc.data.Ibeta;

        /* Seed SMO low-pass filter and raw BEMF states with pre-charged back-EMF vector values
         * to prevent transients upon entering closed-loop. */
        g_foc.ctrl.smo.Ealpha = -E_bemf * sin_th;
        g_foc.ctrl.smo.Ebeta = E_bemf * cos_th;
        g_foc.ctrl.smo.Ealpha_flt = -E_bemf * sin_th;
        g_foc.ctrl.smo.Ebeta_flt = E_bemf * cos_th;

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = omega_now;
            if (fabsf(g_foc.cmd.speed_ref_target) < 1.0f ||
                g_foc.cmd.speed_ref_target < omega_now) {
                g_foc.cmd.speed_ref_target = omega_now;
            }
            float iq_hand_off = g_foc.cfg.startup_current;
            float iq_min = 0.8f;
            float iq_max = g_foc.cfg.motor_max_curr * 0.15f;
            if (iq_max < iq_min) iq_max = iq_min;
            iq_hand_off = clampf(iq_hand_off, iq_min, iq_max);

            g_foc.cmd.Iq_ref = iq_hand_off;
            g_foc.data.Iq_ref_cmd = iq_hand_off;
            LADRC_SeedState(&g_foc.ctrl.speed, omega_now, iq_hand_off);
        } else {
            g_foc.cmd.Iq_ref = 0.0f;
            g_foc.data.Iq_ref_cmd = 0.0f;
        }
        g_foc.cmd.Id_ref = 0.0f;
        if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            float max_v = SQRT3_INV * 2.0f * (MAX_DUTY_HIGH - 0.5f) * g_foc.data.Vbus;
            float E_bemf_norm = (max_v > 1.0f) ? (E_bemf / max_v) : 0.0f;
            E_bemf_norm = clampf(E_bemf_norm, -1.0f, 1.0f);
            g_foc.cmd.Vq_ref = E_bemf_norm;
        }

        /* Transition blending: bypass for flying start since the observer is already
         * locked and aligned. Angle blending can cause phase drift and current spikes. */
        g_foc.startup.theta = theta_now;
        g_foc.startup.omega = omega_now;
        s_blend_alpha = 1.0f;
        s_transition_counter = 0;
        s_transition_samples = 0;
        s_in_transition = 0;

        FOC_SetPhaseVoltageDMA(0);
        FOC_EnableDrivers(1);
        g_foc.status.state = FOC_STATE_RUN;
    }
}

static void FOC_StateAlign(void) {
    g_foc.startup.counter++;
    if (g_foc.startup.counter == 1) {
        /* Calculate dynamic startup PI gains based on 4x Handoff Speed and apply them once at start
         * of Align */
        float handoff_omega = g_foc.cfg.startup_handoff_speed * RPM_TO_RAD * g_foc.cfg.motor_poles;
        float bw_startup = 4.0f * handoff_omega;
        float kp_startup = g_foc.cfg.motor_ls * bw_startup;
        float ki_startup = g_foc.cfg.motor_rs * bw_startup;
        PI_SetGains(&g_foc.ctrl.id, kp_startup, ki_startup);
        PI_SetGains(&g_foc.ctrl.iq, kp_startup, ki_startup);
    }

    float align_samples = (float)ALIGN_DURATION_MS * 0.001f * (float)CONTROL_FREQUENCY;
    float progress = (float)g_foc.startup.counter / align_samples;
    if (progress > 1.0f) progress = 1.0f;

    /* 1. Ramp alignment current from 0 to align_current over the first 30% of alignment duration */
    float ramp_duration = 0.3f;
    if (progress < ramp_duration) {
        g_foc.cmd.Id_ref = (progress / ramp_duration) * g_foc.cfg.align_current;
    } else {
        g_foc.cmd.Id_ref = g_foc.cfg.align_current;
    }
    g_foc.cmd.Iq_ref = 0.0f;

    /* 2. Smooth angle sweep from -45 degrees (-0.25 normalized) to 0 degrees (0.0 normalized)
     * to break any 180-degree electrical angle dead spots without mechanical impact. */
    float start_angle = -0.25f;  // -45 degrees electrical
    g_foc.data.theta_elec = start_angle * (1.0f - progress);

    g_foc.data.omega_elec = 0.0f;
    g_foc.data.speed_rpm = 0.0f;

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);
    park_transform(g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error);
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error);

    svpwm_calculate(g_foc.data.theta_elec);

    if (g_foc.startup.counter > (uint32_t)align_samples) {
        g_foc.startup.counter = 0;
        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        g_foc.ctrl.smo.theta_est = 0.0f;
        g_foc.ctrl.smo.omega_est = 0.0f;
        g_foc.ctrl.smo.omega_out = 0.0f;
        g_foc.ctrl.smo.omega_stf = 0.0f;
        g_foc.ctrl.smo.pll_integral = 0.0f;
        g_foc.status.state = FOC_STATE_STARTUP;
    }
}

static void FOC_StateStartup(void) {
    g_foc.startup.counter++;

    float accel_rad = g_foc.cfg.startup_accel * RPM_TO_RAD * g_foc.cfg.motor_poles;
    g_foc.startup.omega += accel_rad * CONTROL_PERIOD;

    float handoff_omega = g_foc.cfg.startup_handoff_speed * RPM_TO_RAD * g_foc.cfg.motor_poles;
    if (g_foc.startup.omega > handoff_omega) {
        g_foc.startup.omega = handoff_omega;
    }

    g_foc.data.omega_elec = g_foc.startup.omega;
    g_foc.data.speed_rpm = (g_foc.startup.omega / g_foc.cfg.motor_poles) * (60.0f / TWO_PI);

    g_foc.startup.theta += (g_foc.startup.omega * CONTROL_PERIOD) / PI;
    g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);
    g_foc.data.theta_elec = g_foc.startup.theta;

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);
    park_transform(g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    g_foc.cmd.Id_ref = 0.0f;

    float startup_ramp_samples = 0.3f * (float)CONTROL_FREQUENCY;
    if ((float)g_foc.startup.counter < startup_ramp_samples) {
        float ratio = (float)g_foc.startup.counter / startup_ramp_samples;
        g_foc.cmd.Iq_ref = ratio * g_foc.cfg.startup_current;
    } else {
        g_foc.cmd.Iq_ref = g_foc.cfg.startup_current;
    }

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    /* Calculate voltage commands using closed-loop current PI controllers + Feedforward */
    float omega_e = g_foc.startup.omega;
    float omega_Ls = omega_e * g_foc.cfg.motor_ls;
    float E_bemf = omega_e * g_foc.cfg.motor_flux;

    g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq;
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + omega_Ls * g_foc.data.Id + E_bemf;

    /* Clamp (Vd, Vq) vector magnitude to 0.57735 * Vbus (SVPWM linear limit) */
    float max_v = SQRT3_INV * g_foc.data.Vbus;
    float v_sq = g_foc.data.Vd * g_foc.data.Vd + g_foc.data.Vq * g_foc.data.Vq;
    if (v_sq > max_v * max_v && v_sq > 1e-6f) {
        float inv_v = max_v / sqrtf(v_sq);
        g_foc.data.Vd *= inv_v;
        g_foc.data.Vq *= inv_v;
    }

    /* Inverse Park */
    svpwm_calculate(g_foc.data.theta_elec);

    SMO_Update(&g_foc.ctrl.smo, g_foc.data.Valpha, g_foc.data.Vbeta, g_foc.data.Ialpha,
               g_foc.data.Ibeta);

#if ENABLE_CLOSED_LOOP_HANDOFF
    if (g_foc.startup.omega >= handoff_omega * 0.9f) {
        SMO_ResetStates(&g_foc.ctrl.smo, g_foc.startup.omega);
        SMO_SlowTask(&g_foc.ctrl.smo);

        /* Restore original default PI gains from config */
        PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
        PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = g_foc.startup.omega;
            LADRC_SeedState(&g_foc.ctrl.speed, g_foc.startup.omega, g_foc.cmd.Iq_ref);
        } else {
            g_foc.cmd.Iq_ref = g_foc.data.Iq;
        }
        g_foc.data.Iq_ref_cmd = g_foc.cmd.Iq_ref;

        if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            float max_v = SQRT3_INV * g_foc.data.Vbus;
            float Vq_norm = (max_v > 1.0f) ? (g_foc.data.Vq / max_v) : 0.0f;
            Vq_norm = clampf(Vq_norm, 0.0f, 1.0f);
            g_foc.cmd.Vq_ref = Vq_norm;
            g_foc.cmd.Vq_ref_target =
                (Vq_norm > g_foc.cmd.Vq_ref_target) ? Vq_norm : g_foc.cmd.Vq_ref_target;
        }
        s_blend_alpha = 0.0f;
        s_transition_counter = 0;
        s_transition_samples = (uint32_t)(TRANSITION_BLEND_MS * 0.001f * (float)CONTROL_FREQUENCY);
        if (s_transition_samples < 1) s_transition_samples = 1;
        s_in_transition = 1;
        g_foc.status.in_transition = 1;

        g_foc.cmd.Id_ref = 0.0f;
        g_foc.status.state = FOC_STATE_RUN;
    }
#endif

    /* Timeout check (only if enabled) */
#if STARTUP_TIMEOUT_MS > 0
    if (g_foc.startup.counter > (uint32_t)((STARTUP_TIMEOUT_MS * CONTROL_FREQUENCY) / 1000)) {
        g_foc.status.fault = FOC_FAULT_STARTUP_FAIL;
        g_foc.status.state = FOC_STATE_FAULT;
    }
#endif
}

CCMRAM_FUNC static void FOC_StateRun(void) {
    s_closed_loop_counter++;

    SMO_Update(&g_foc.ctrl.smo, g_foc.data.Valpha, g_foc.data.Vbeta, g_foc.data.Ialpha,
               g_foc.data.Ibeta);

    float smo_theta = SMO_GetAngle(&g_foc.ctrl.smo);
    float smo_omega = SMO_GetSpeed(&g_foc.ctrl.smo);
    float smo_speed_rpm = SMO_GetSpeedRPM(&g_foc.ctrl.smo);

    if (s_in_transition) {
        s_transition_counter++;
        s_blend_alpha = (float)s_transition_counter / (float)s_transition_samples;

        if (s_blend_alpha >= 1.0f) {
            s_blend_alpha = 1.0f;
            s_in_transition = 0;
            g_foc.status.in_transition = 0;

            /* Restore original default PI gains from config */
            PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
            PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);
        }

        g_foc.startup.theta += (g_foc.startup.omega * CONTROL_PERIOD) / PI;
        g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);
        float delta = normalize_angle_norm(smo_theta - g_foc.startup.theta);
        g_foc.data.theta_elec = normalize_angle_norm(g_foc.startup.theta + s_blend_alpha * delta);
        g_foc.data.omega_elec =
            (1.0f - s_blend_alpha) * g_foc.startup.omega + s_blend_alpha * smo_omega;
        g_foc.data.speed_rpm = (1.0f - s_blend_alpha) * (g_foc.startup.omega /
                                                         g_foc.cfg.motor_poles * (60.0f / TWO_PI)) +
                               s_blend_alpha * smo_speed_rpm;
    } else {
        g_foc.data.theta_elec = smo_theta;
        g_foc.data.omega_elec = smo_omega;
        g_foc.data.speed_rpm = smo_speed_rpm;
    }

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);

    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    if (g_foc.ctrl.bist.mode != BIST_MODE_OFF) {
        BIST_Update(&g_foc.ctrl.bist, (float*)&g_foc.cmd.Iq_ref);
    }

    g_foc.cmd.Id_ref = 0.0f;
    float target_iq = saturatef(g_foc.cmd.Iq_ref, g_foc.cfg.motor_max_curr);
    float max_diq = 1000.0f * CONTROL_PERIOD;
    g_foc.data.Iq_ref_cmd += clampf(target_iq - g_foc.data.Iq_ref_cmd, -max_diq, max_diq);

    /* Real-time Magnetic Saturation Gain Scheduling (reuse l_ratio from SMO_Update) */
    float l_ratio = g_foc.ctrl.smo.l_ratio;
    g_foc.ctrl.id.Kp = g_foc.cfg.kp_id * l_ratio;
    g_foc.ctrl.iq.Kp = g_foc.cfg.kp_iq * l_ratio;

    float ff_gain = 0.5f;
    float omega_Ls = ff_gain * g_foc.data.omega_elec * (g_foc.cfg.motor_ls * l_ratio);
    float E_bemf = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_flux;
    float max_v = SQRT3_INV * 2.0f * (MAX_DUTY_HIGH - 0.5f) * g_foc.data.Vbus;

    if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
        float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
        g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq;
        float vd_abs = fabsf(g_foc.data.Vd);
        if (vd_abs >= max_v) {
            g_foc.data.Vd = (g_foc.data.Vd > 0.0f) ? max_v : -max_v;
            g_foc.data.Vq = 0.0f;
        } else {
            float vq_max = sqrtf(max_v * max_v - g_foc.data.Vd * g_foc.data.Vd);
            float vq_cmd = g_foc.cmd.Vq_ref * max_v;
            g_foc.data.Vq = clampf(vq_cmd, -vq_max, vq_max);
        }
    } else {
        float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
        float Iq_error = g_foc.data.Iq_ref_cmd - g_foc.data.Iq;
        g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq;
        g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + omega_Ls * g_foc.data.Id + E_bemf;
    }

    float theta_adv = g_foc.data.theta_elec;
    if (g_foc.cfg.comp_delay_samples > 0.001f) {
        theta_adv += g_foc.data.omega_elec * (g_foc.cfg.comp_delay_samples * CONTROL_PERIOD) / PI;
        theta_adv = normalize_angle_norm(theta_adv);
    }

    svpwm_calculate(theta_adv);

    /*=======================================================================*/
    /* Ramp-down stop check                                                  */
    /*=======================================================================*/
    if (s_stopping) {
        s_stop_counter++;
        float abs_speed = fabsf(g_foc.data.speed_rpm);
        float abs_iq = fabsf(g_foc.data.Iq);
        uint32_t timeout = (uint32_t)(STOP_TIMEOUT_MS * 0.001f * (float)CONTROL_FREQUENCY);
        float stop_current_thr = g_foc.cfg.motor_max_curr * (STOP_CURRENT_PERCENT / 100.0f);

        if ((abs_speed < g_foc.cfg.motor_min_spd && abs_iq < stop_current_thr &&
             g_foc.cmd.speed_ref == 0.0f) ||
            s_stop_counter >= timeout) {
            FOC_EnableDrivers(0);
            g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;
            g_foc.cmd.Iq_ref = 0.0f;
            g_foc.data.Iq_ref_cmd = 0.0f;
            g_foc.cmd.Vq_ref = 0.0f;
            g_foc.cmd.Vq_ref_target = 0.0f;
            g_foc.cmd.speed_ref = 0.0f;
            PI_Reset(&g_foc.ctrl.id);
            PI_Reset(&g_foc.ctrl.iq);
            LADRC_Reset(&g_foc.ctrl.speed);
            s_stopping = 0;
            s_stop_counter = 0;
            g_foc.cmd.speed_ref_target = s_saved_speed_target;
            g_foc.cmd.Iq_ref_target = s_saved_iq_target;
            g_foc.status.state = FOC_STATE_IDLE;
        }
    }
}

static void FOC_StateStop(void) {
    FOC_SetPhaseVoltageDMA(0);
    FOC_EnableDrivers(0);
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
    g_foc.data.Iq_ref_cmd = 0.0f;
    if (g_foc.status.state != FOC_STATE_FAULT) {
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

static void FOC_StateFault(void) {
    FOC_SetPhaseVoltageDMA(0);
    FOC_EnableDrivers(0);
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
}

static void FOC_StateSelfCommission(void) {
    g_foc.data.theta_elec = 0.0f;
    float cos_th = 1.0f;
    float sin_th = 0.0f;

    /* Reconstruct currents and transform to d-q frame */
    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    /* Run the d-axis AC parameter identification step */
    MotorID_RunStep(g_foc.data.Id, g_foc.data.Iq, g_foc.data.Vbus, &g_foc.data.Vd, &g_foc.data.Vq);

    /* Inverse Park and SVPWM calculation */
    svpwm_calculate(0.0f);

    if (id_result.state == MOTOR_ID_STATE_COMPLETE) {
        MotorID_Stop();
        FOC_StateStop();
    } else if (id_result.state == MOTOR_ID_STATE_ERROR) {
        FOC_StateStop();
    }
}

/*===========================================================================*/
/* API Functions                                                             */
/*===========================================================================*/

void FOC_SetSpeedRef(float speed_rpm) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    g_foc.cmd.speed_ref_target = speed_rpm * RPM_TO_RAD * g_foc.cfg.motor_poles;
}

void FOC_SetTorqueRef(float torque_percent) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    g_foc.cmd.Iq_ref_target = (torque_percent / 100.0f) * g_foc.cfg.motor_max_curr;
}

void FOC_SetVoltageRef(float voltage_percent) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    float pct = clampf(voltage_percent, 0.0f, 100.0f);
    g_foc.cmd.Vq_ref_target = pct / 100.0f;
}

void FOC_SetControlMode(FOC_ControlMode_t mode) {
    g_foc.status.control_mode = mode;
}

FOC_State_t FOC_GetState(void) {
    return g_foc.status.state;
}

FOC_Fault_t FOC_GetFault(void) {
    return g_foc.status.fault;
}

void FOC_ClearFault(void) {
    if (g_foc.status.state == FOC_STATE_FAULT) {
        // Clear all hardware AWD flags before transitioning out of FAULT state
        LL_ADC_ClearFlag_AWD1(ADC1);
        LL_ADC_ClearFlag_AWD2(ADC1);
        LL_ADC_ClearFlag_AWD1(ADC2);
        LL_ADC_ClearFlag_AWD2(ADC2);

        g_foc.status.fault = FOC_FAULT_NONE;
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

void FOC_EnableDrivers(uint8_t enable) {
    if (enable) {
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                          LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                          LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
        LL_TIM_EnableAllOutputs(TIM1);
    } else {
        LL_TIM_DisableAllOutputs(TIM1);
        LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                           LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                           LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    }
}

void FOC_EnableDriver(uint8_t phase, uint8_t enable) {
    switch (phase) {
        case 1:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
            }
            break;
        case 2:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
            }
            break;
        case 3:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
            }
            break;
    }
}

void FOC_StartSelfCommission(void) {
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_STOP) {
        FOC_Start();
        MotorID_Start();
        FOC_EnableDrivers(0);
    }
}

void FOC_SetDirection(int8_t dir) {
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_STOP ||
        g_foc.status.state == FOC_STATE_FAULT) {
        g_foc.status.reverse = (dir < 0) ? -1.0f : 1.0f;
    }
}

int8_t FOC_GetDirection(void) {
    return (g_foc.status.reverse < 0.0f) ? -1 : 1;
}

void playTune() {
    uint32_t cycle_tick = g_foc.status.run_counter % BEEP_PERIOD_TICKS;
    uint32_t b1_end = BEEP_DURATION_TICKS;
    uint32_t b2_start = BEEP_DURATION_TICKS * 2;
    uint32_t b2_end = BEEP_DURATION_TICKS * 3;

    if ((cycle_tick < b1_end) || (cycle_tick >= b2_start && cycle_tick < b2_end)) {
        static float tune_phase = 0.0f;
        tune_phase += BEEP_STEP_FREQ;
        if (tune_phase >= 1.0f) tune_phase -= 2.0f;

        float cos_out, sin_out;
        cordic_sincos(tune_phase, &cos_out, &sin_out);

        g_foc.data.duty_a = 0.5f + sin_out * 0.05f;
        g_foc.data.duty_b = 0.5f - sin_out * 0.05f;
        g_foc.data.duty_c = 0.5f;
    } else {
        g_foc.data.duty_a = 0.5f;
        g_foc.data.duty_b = 0.5f;
        g_foc.data.duty_c = 0.5f;
    }
}