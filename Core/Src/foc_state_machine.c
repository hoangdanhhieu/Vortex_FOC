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

/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/

void FOC_StartSelfCommission(void);

/* Calibration samples count */
#define CAL_SAMPLES 256

/*===========================================================================*/
/* Global Instance                                                           */
/*===========================================================================*/

FOC_Control_t g_foc __attribute__((aligned(4))); /* aligned for efficient FPU access */
static volatile uint8_t foc_initialized = 0;

static float s_Id_filt = 0.0f;
static float s_Iq_filt = 0.0f;

/* Safety check state */
static uint8_t oc_count = 0;            /* Overcurrent deglitch counter */
static uint32_t stall_counter = 0;      /* Stall timer (ISR ticks) */
static uint32_t ov_count = 0;           /* Overvoltage deglitch counter */
static uint32_t uv_count = 0;           /* Undervoltage deglitch counter */
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

/* Phase voltage calibration & spinning detection accumulators */
static int32_t s_vphase_a_accum = 0;
static int32_t s_vphase_b_accum = 0;
static int32_t s_vphase_c_accum = 0;

static uint16_t s_vphase_a_min = 4096;
static uint16_t s_vphase_a_max = 0;
static uint16_t s_vphase_b_min = 4096;
static uint16_t s_vphase_b_max = 0;
static uint16_t s_vphase_c_min = 4096;
static uint16_t s_vphase_c_max = 0;

extern volatile uint32_t adc_isr_us;
extern volatile uint16_t adc_regular_buffer[];
extern volatile float ADC_Vref;

/**
 * @brief Normalize angle to [-1, 1) range (corresponds to [-pi, pi))
 */
CCMRAM_FUNC static inline float normalize_angle_norm(float angle) {
    angle += 1.0f;
    angle -= 2.0f * (float)((int)(angle * 0.5f));
    if (angle >= 2.0f) angle -= 2.0f;
    if (angle < 0.0f) angle += 2.0f;
    return angle - 1.0f;
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
    /* Block ISR state machine until init completes */
    foc_initialized = 0;

    /* Clear control structure */
    g_foc.status.state = FOC_STATE_IDLE;
    g_foc.status.control_mode = FOC_MODE_SPEED;
    g_foc.status.fault = FOC_FAULT_NONE;

    /* Clear measurements */
    g_foc.data.Ia = g_foc.data.Ib = g_foc.data.Ic = 0.0f;
    g_foc.data.Ialpha = g_foc.data.Ibeta = 0.0f;
    g_foc.data.Id = g_foc.data.Iq = 0.0f;
    g_foc.data.Vd = g_foc.data.Vq = 0.0f;
    g_foc.data.Valpha = g_foc.data.Vbeta = 0.0f;
    g_foc.data.Vbus = 12.0f;

    /* Clear references */
    g_foc.cmd.Id_ref = 0.0f;
    g_foc.cmd.Iq_ref = 0.0f;
    g_foc.cmd.speed_ref = 0.0f;

    /* Clear estimates */
    g_foc.data.theta_elec = 0.0f;
    g_foc.data.omega_elec = 0.0f;
    g_foc.data.speed_rpm = 0.0f;

    /* Initialize startup variables */
    g_foc.startup.theta = 0.0f;
    g_foc.startup.omega = 0.0f;
    g_foc.startup.counter = 0;

    /* Initialize duty cycles */
    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    /* Initialize PI controllers with compile-time defaults. */
    PI_Init(&g_foc.ctrl.id, PI_ID_KP, PI_ID_KI, -12.0f, 12.0f, CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.id, -12.0f, 12.0f);

    PI_Init(&g_foc.ctrl.iq, PI_IQ_KP, PI_IQ_KI, -12.0f, 12.0f, CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.iq, -12.0f, 12.0f);

    PI_Init(&g_foc.ctrl.speed, PI_SPEED_KP, PI_SPEED_KI, PI_SPEED_OUT_MIN, PI_SPEED_OUT_MAX,
            CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.speed, PI_SPEED_INT_MIN, PI_SPEED_INT_MAX);

    /* Initialize SMO observer */
    SMO_Init(&g_foc.ctrl.smo);

    /* Initialize PID Profiler */
    BIST_Init(&g_foc.ctrl.bist);

    /* Initialize ADC calibration */
    g_foc.adc_cal.offset_a = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_b = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_c = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_vphase_a = 234;
    g_foc.adc_cal.offset_vphase_b = 234;
    g_foc.adc_cal.offset_vphase_c = 234;
    g_foc.adc_cal.cal_samples = 0;

    /* Statistics */
    g_foc.status.run_counter = 0;
    g_foc.isr_time_cycles = 0;

    /* Disable drivers initially */
    FOC_EnableDrivers(0);

    /* Initialize Motor ID */
    MotorID_Init();

    g_foc.status.reverse = 1.0f;

    /* Runtime config defaults (will be overwritten by FlashConfig_Apply) */
    g_foc.cfg.startup_current = STARTUP_CURRENT;
    g_foc.cfg.align_current = ALIGN_CURRENT;
    g_foc.cfg.startup_accel = STARTUP_ACCEL;
    g_foc.cfg.startup_handoff_speed = STARTUP_HANDOFF_SPEED;
    g_foc.cfg.speed_ramp_accel = SPEED_RAMP_ACCEL;
    g_foc.cfg.speed_ramp_decel = SPEED_RAMP_DECEL;
    g_foc.cfg.fault_oc_threshold = FAULT_OVERCURRENT_THRESHOLD;
    g_foc.cfg.fault_oc_count = (uint8_t)FAULT_OVERCURRENT_COUNT;
    g_foc.cfg.fault_ov_threshold = FAULT_OVERVOLTAGE_THRESHOLD;
    g_foc.cfg.fault_uv_threshold = FAULT_UNDERVOLTAGE_THRESHOLD;
    g_foc.cfg.fault_stall_enable = (uint8_t)FAULT_STALL_ENABLE;
    g_foc.cfg.fault_stall_speed = FAULT_STALL_SPEED_RPM;
    g_foc.cfg.fault_stall_current = FAULT_STALL_CURRENT_A;
    g_foc.cfg.fault_stall_time_ms = FAULT_STALL_TIME_MS;

    /* Allow ISR to process state machine */
    foc_initialized = 1;
}

void FOC_Start(void) {
    if (g_foc.status.state == FOC_STATE_IDLE) {
        /* Reset controllers */
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        PI_Reset(&g_foc.ctrl.speed);
        SMO_Reset(&g_foc.ctrl.smo);

        s_Id_filt = 0.0f;
        s_Iq_filt = 0.0f;

        /* Reset transition state */
        s_blend_alpha = 0.0f;
        s_transition_counter = 0;
        s_in_transition = 0;

        /* Reset stop state */
        s_stopping = 0;
        s_stop_counter = 0;

        /* Calculate flying start timing from runtime config */
        float f_elec_min = g_foc.cfg.motor_min_spd * g_foc.cfg.motor_poles / 60.0f;
        if (f_elec_min < 1.0f) f_elec_min = 1.0f;
        s_detect_samples = (uint32_t)(2.0f / f_elec_min * (float)CONTROL_FREQUENCY);
        s_lock_samples = (uint32_t)(5.0f / (TWO_PI * SMO_PPL_CUTOFF) * (float)CONTROL_FREQUENCY);
        /* Ensure at least 50 ms for stable PLL lock tracking */
        uint32_t min_lock_samples = (uint32_t)(0.05f * (float)CONTROL_FREQUENCY);  // 50 ms
        if (s_lock_samples < min_lock_samples) s_lock_samples = min_lock_samples;
        s_bemf_threshold = g_foc.cfg.motor_min_spd / g_foc.cfg.motor_kv * 0.5f;
        if (s_bemf_threshold < 0.05f) s_bemf_threshold = 0.05f;

        /* Reset flying start state */
        s_detect_counter = 0;
        s_detect_peak = 0.0f;
        s_flying_start_counter = 0;

        /* Enter calibration state */
        g_foc.adc_cal.offset_a = 0;
        g_foc.adc_cal.offset_b = 0;
        g_foc.adc_cal.offset_c = 0;
        g_foc.adc_cal.cal_samples = 0;

        /* Reset phase voltage calibration accumulators and min/max states */
        s_vphase_a_accum = 0;
        s_vphase_b_accum = 0;
        s_vphase_c_accum = 0;
        s_vphase_a_min = 4096;
        s_vphase_a_max = 0;
        s_vphase_b_min = 4096;
        s_vphase_b_max = 0;
        s_vphase_c_min = 4096;
        s_vphase_c_max = 0;
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
        s_stopping = 1;
        s_stop_counter = 0;
    }
    g_foc.status.state = FOC_STATE_STOP;
}

__attribute__((noinline)) static void FOC_CheckSafety(void) {
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_CALIBRATION ||
        g_foc.status.state == FOC_STATE_FAULT || g_foc.status.state == FOC_STATE_DETECT ||
        g_foc.status.state == FOC_STATE_FLYING_START) {
        oc_count = 0;
        stall_counter = 0;
        ov_count = 0;
        uv_count = 0;
        ground_fault_count = 0;
        return;
    }

    float abs_ia = fabsf(g_foc.data.Ia);
    float abs_ib = fabsf(g_foc.data.Ib);
    float abs_ic = fabsf(g_foc.data.Ic);

    if (abs_ia > g_foc.cfg.fault_oc_threshold || abs_ib > g_foc.cfg.fault_oc_threshold ||
        abs_ic > g_foc.cfg.fault_oc_threshold) {
        if (++oc_count >= g_foc.cfg.fault_oc_count) {
            g_foc.status.fault = FOC_FAULT_OVERCURRENT;
            g_foc.status.state = FOC_STATE_FAULT;
        }
    } else {
        if (oc_count > 0) oc_count--;
    }

    /*--- Bus Voltage protection with Deglitch counters ---*/
    if (g_foc.data.Vbus > g_foc.cfg.fault_ov_threshold) {
        if (++ov_count >= 100) {
            g_foc.status.fault = FOC_FAULT_OVERVOLTAGE;
            g_foc.status.state = FOC_STATE_FAULT;
        }
    } else {
        if (ov_count > 0) ov_count--;
    }

    if (g_foc.data.Vbus < g_foc.cfg.fault_uv_threshold) {
        if (++uv_count >= 500) {
            g_foc.status.fault = FOC_FAULT_UNDERVOLTAGE;
            g_foc.status.state = FOC_STATE_FAULT;
        }
    } else {
        if (uv_count > 0) uv_count--;
    }

    /*--- Ground Fault & Sensor Symmetry Check (Only run when PWM duty cycles are inside sampleable
     * range 15% - 85%) ---*/
    if (g_foc.data.duty_a < 0.85f && g_foc.data.duty_b < 0.85f && g_foc.data.duty_c < 0.85f &&
        g_foc.data.duty_a > 0.15f && g_foc.data.duty_b > 0.15f && g_foc.data.duty_c > 0.15f) {
        float current_sum = g_foc.data.Ia + g_foc.data.Ib + g_foc.data.Ic;
        float gf_threshold = 0.40f * g_foc.cfg.motor_max_curr;
        if (gf_threshold < 1.0f) gf_threshold = 1.0f;

        if (fabsf(current_sum) > gf_threshold) {
            if (++ground_fault_count >= 500) {
                g_foc.status.fault = FOC_FAULT_GROUND;
                g_foc.status.state = FOC_STATE_FAULT;
            }
        } else {
            if (ground_fault_count > 0) ground_fault_count--;
        }
    } else {
        if (ground_fault_count > 0) ground_fault_count--;
    }

    /*--- Stall / Loss of Sync detection ---*/
    if (g_foc.cfg.fault_stall_enable && g_foc.status.state == FOC_STATE_RUN) {
        // Use user-configured fault_stall_current from GUI as estimation error threshold
        float threshold_c = g_foc.cfg.fault_stall_current;
        float threshold_c_sq = threshold_c * threshold_c;

        if (g_foc.ctrl.smo.current_err_sq > threshold_c_sq) {
            stall_counter++;
            if (stall_counter >
                (uint32_t)(g_foc.cfg.fault_stall_time_ms / (CONTROL_PERIOD * 1000.0f))) {
                g_foc.status.fault = FOC_FAULT_STALL;
                g_foc.status.state = FOC_STATE_FAULT;
            }
        } else {
            if (stall_counter > 0) stall_counter--;
        }
    } else {
        stall_counter = 0;
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
CCMRAM_FUNC void FOC_HighFrequencyTask(uint16_t adc_ia, uint16_t adc_ib, uint16_t adc_ic,
                                       uint16_t adc_vbus) {
    if (!foc_initialized) return;
    if (g_foc.status.state != FOC_STATE_RUN) {
        s_closed_loop_counter = 0;
        g_foc.ctrl.smo.enable_harmonic_comp = 0;
    }
    g_foc.data.Vbus = foc_adc_to_vbus(adc_vbus);
    if (g_foc.data.Vbus < 1.0f) g_foc.data.Vbus = 1.0f;
    g_foc.data.Vbus_inv = 1.0f / g_foc.data.Vbus;
    g_foc.data.Ia = ((float)adc_ia - (float)g_foc.adc_cal.offset_a) * ADC_TO_CURRENT;
    g_foc.data.Ib = ((float)adc_ib - (float)g_foc.adc_cal.offset_b) * ADC_TO_CURRENT;
    g_foc.data.Ic = ((float)adc_ic - (float)g_foc.adc_cal.offset_c) * ADC_TO_CURRENT;
    float noise = (g_foc.data.Ia + g_foc.data.Ib + g_foc.data.Ic) * 0.33333333f;
    g_foc.data.Ia -= noise;
    g_foc.data.Ib -= noise;
    g_foc.data.Ic -= noise;
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, adc_ia);
    g_foc.status.run_counter++;

    FOC_CheckSafety();

    if (g_foc.status.reverse < 0.0f) {
        float t = g_foc.data.Ib;
        g_foc.data.Ib = g_foc.data.Ic;
        g_foc.data.Ic = t;
    }

    clarke_transform(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Ic, &g_foc.data.Ialpha,
                     &g_foc.data.Ibeta);

    if (g_foc.status.state == FOC_STATE_DETECT || g_foc.status.state == FOC_STATE_FLYING_START) {
        g_foc.data.Vphase_a =
            foc_adc_to_vphase(adc_regular_buffer[1], g_foc.adc_cal.offset_vphase_a);
        g_foc.data.Vphase_b =
            foc_adc_to_vphase(adc_regular_buffer[2], g_foc.adc_cal.offset_vphase_b);
        g_foc.data.Vphase_c =
            foc_adc_to_vphase(adc_regular_buffer[3], g_foc.adc_cal.offset_vphase_c);
    }

    switch (g_foc.status.state) {
        case FOC_STATE_IDLE:
            FOC_StateIdle();
            break;
        case FOC_STATE_CALIBRATION:
            g_foc.adc_cal.offset_a += adc_ia;
            g_foc.adc_cal.offset_b += adc_ib;
            g_foc.adc_cal.offset_c += adc_ic;
            {
                uint16_t va = adc_regular_buffer[1];
                uint16_t vb = adc_regular_buffer[2];
                uint16_t vc = adc_regular_buffer[3];
                s_vphase_a_accum += va;
                s_vphase_b_accum += vb;
                s_vphase_c_accum += vc;
                if (va < s_vphase_a_min) s_vphase_a_min = va;
                if (va > s_vphase_a_max) s_vphase_a_max = va;
                if (vb < s_vphase_b_min) s_vphase_b_min = vb;
                if (vb > s_vphase_b_max) s_vphase_b_max = vb;
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
            g_foc.plot.Vd = g_foc.data.Vd;
            g_foc.plot.Vq = g_foc.data.Vq;
            g_foc.plot.Id = g_foc.data.Id;
            g_foc.plot.Iq = g_foc.data.Iq;
            g_foc.plot.Iq_ref = g_foc.cmd.Iq_ref;
            g_foc.plot.theta_elec = g_foc.data.theta_elec;
            g_foc.plot.Ia = g_foc.data.Ia;
            g_foc.plot.Ib = g_foc.data.Ib;
            g_foc.plot.Ic = g_foc.data.Ic;
            g_foc.plot.duty_a = g_foc.data.duty_a;
            g_foc.plot.duty_b = g_foc.data.duty_b;
            g_foc.plot.duty_c = g_foc.data.duty_c;
            g_foc.plot.ready = 1;
        }
    }

    /* Centralized deadtime compensation applied to local copies.
     * g_foc.data.duty_a/b/c remain CLEAN (no comp embedded), so ramp
     * controllers in Motor ID and other state machines never accumulate
     * the compensation offset across ISR cycles. */
    float out_a = g_foc.data.duty_a;
    float out_b = g_foc.data.duty_b;
    float out_c = g_foc.data.duty_c;

    FOC_ApplyDeadtimeCompensation(&out_a, &out_b, &out_c);

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
}

static void FOC_StateCalibration(void) {
    if (g_foc.adc_cal.cal_samples >= CAL_SAMPLES) {
        g_foc.adc_cal.offset_a /= CAL_SAMPLES;
        g_foc.adc_cal.offset_b /= CAL_SAMPLES;
        g_foc.adc_cal.offset_c /= CAL_SAMPLES;

        /* Check if peak-to-peak variation on any phase is above threshold (30 ADC counts)
         * to detect if the motor is already spinning. */
        uint16_t diff_a = s_vphase_a_max - s_vphase_a_min;
        uint16_t diff_b = s_vphase_b_max - s_vphase_b_min;
        uint16_t diff_c = s_vphase_c_max - s_vphase_c_min;

        uint8_t is_spinning = (diff_a > 30) || (diff_b > 30) || (diff_c > 30);

        if (!is_spinning) {
            g_foc.adc_cal.offset_vphase_a = s_vphase_a_accum / CAL_SAMPLES;
            g_foc.adc_cal.offset_vphase_b = s_vphase_b_accum / CAL_SAMPLES;
            g_foc.adc_cal.offset_vphase_c = s_vphase_c_accum / CAL_SAMPLES;
        }

        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        g_foc.startup.counter = 0;

        if (id_result.state != MOTOR_ID_STATE_ALIGN) {
            g_foc.status.state = FOC_STATE_DETECT;
        } else {
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
    if (amp > s_detect_peak) s_detect_peak = amp;

    if (s_detect_counter >= s_detect_samples) {
        float thr_sq = s_bemf_threshold * s_bemf_threshold;
        if (s_detect_peak > thr_sq) {
            SMO_Reset(&g_foc.ctrl.smo);

            /* Seed speed and angle from the last measured back-EMF peak/vector */
            float E_amp_real = sqrtf(s_detect_peak) * TWO_THIRDS; /* scale back by 1.5 */
            float est_omega = E_amp_real / g_foc.cfg.motor_flux;
            if (est_omega > SMO_PLL_INT_MAX) est_omega = SMO_PLL_INT_MAX;

            /* Match the direction of target speed */
            if (g_foc.cmd.speed_ref_target < 0.0f) {
                est_omega = -est_omega;
            }

            g_foc.ctrl.smo.omega_est = est_omega;
            g_foc.ctrl.smo.pll_integral = est_omega;

            float theta_init = cordic_atan2(-E_alpha, E_beta);
            g_foc.ctrl.smo.theta_est = theta_init;

            s_flying_start_counter = 0;
            g_foc.status.state = FOC_STATE_FLYING_START;
        } else {
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

    if (s_flying_start_counter < s_lock_samples) {
        /* Normal flying start: drivers OFF, output 0V, PLL tracking */
        g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    } else if (s_flying_start_counter == s_lock_samples) {
        /* -----------------------------------------------------------
         * PRECHARGE CYCLE: Compute correct SVPWM duties that match
         * back-EMF. These values go to the CCR preload register via
         * foc_set_pwm_duty() at the end of this ISR. The timer update
         * event (at counter underflow) will transfer them to the shadow
         * register BEFORE the next ISR fires.
         *
         * DO NOT enable drivers here — the shadow register still has
         * the old 0.5 duty from previous cycles!
         * ----------------------------------------------------------- */
        float omega_now = g_foc.ctrl.smo.omega_est;
        float theta_now = SMO_GetAngle(&g_foc.ctrl.smo);
        float E_bemf = omega_now * g_foc.cfg.motor_flux;

        g_foc.data.Vd = 0.0f;
        g_foc.data.Vq = E_bemf;

        float sin_th, cos_th;
        cordic_sincos(theta_now, &cos_th, &sin_th);
        inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, cos_th, sin_th, &g_foc.data.Valpha,
                               &g_foc.data.Vbeta);
        svpwm_calculate(); /* duty_a/b/c now match back-EMF */

    } else {
        /* -----------------------------------------------------------
         * ENABLE CYCLE: The timer update event between the previous
         * ISR and this one has loaded the correct duty from the
         * precharge cycle into the CCR shadow register. The PWM output
         * NOW matches back-EMF. Safe to enable gate drivers — the
         * very first PWM pulse the motor sees is correct.
         * ----------------------------------------------------------- */
        float omega_now = g_foc.ctrl.smo.omega_est;
        float theta_now = SMO_GetAngle(&g_foc.ctrl.smo);
        float E_bemf = omega_now * g_foc.cfg.motor_flux;

        /* Recompute SVPWM for the updated angle (one cycle later) */
        g_foc.data.Vd = 0.0f;
        g_foc.data.Vq = E_bemf;

        float sin_th, cos_th;
        cordic_sincos(theta_now, &cos_th, &sin_th);
        inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, cos_th, sin_th, &g_foc.data.Valpha,
                               &g_foc.data.Vbeta);
        svpwm_calculate();

        /* Pre-load current PI integrals for bumpless transfer */
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        PI_Reset(&g_foc.ctrl.speed);
        g_foc.ctrl.iq.integral = E_bemf; /* Vq starts at back-EMF level */
        g_foc.ctrl.id.integral = 0.0f;

        s_Id_filt = 0.0f;
        s_Iq_filt = 0.0f;

        /* Seed SMO current observer with actual measured currents */
        g_foc.ctrl.smo.Ialpha_est = g_foc.data.Ialpha;
        g_foc.ctrl.smo.Ibeta_est = g_foc.data.Ibeta;

        /* Seed SMO low-pass filter states with pre-charged back-EMF vector values
         * to prevent transients upon entering closed-loop. */
        g_foc.ctrl.smo.Ealpha_flt = -E_bemf * sin_th;
        g_foc.ctrl.smo.Ebeta_flt = E_bemf * cos_th;

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = omega_now;
            g_foc.ctrl.speed.integral = 0.0f;
        }
        g_foc.cmd.Iq_ref = 0.0f;
        g_foc.cmd.Id_ref = 0.0f;

        /* Transition blending: bypass for flying start since the observer is already
         * locked and aligned. Angle blending can cause phase drift and current spikes. */
        g_foc.startup.theta = theta_now;
        g_foc.startup.omega = omega_now;
        s_blend_alpha = 1.0f;
        s_transition_counter = 0;
        s_transition_samples = 0;
        s_in_transition = 0;

        FOC_EnableDrivers(1);
        g_foc.status.state = FOC_STATE_RUN;
    }
}

static void FOC_StateAlign(void) {
    g_foc.startup.counter++;

    g_foc.data.theta_elec = 0.0f;
    g_foc.data.omega_elec = 0.0f;
    g_foc.data.speed_rpm = 0.0f;

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);
    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    g_foc.cmd.Id_ref = g_foc.cfg.align_current;
    g_foc.cmd.Iq_ref = 0.0f;

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error);
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error);

    inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, cos_th, sin_th, &g_foc.data.Valpha,
                           &g_foc.data.Vbeta);

    svpwm_calculate();

    float align_samples = (float)ALIGN_DURATION_MS * 0.001f * (float)CONTROL_FREQUENCY;
    if (g_foc.startup.counter > (uint32_t)align_samples) {
        g_foc.startup.counter = 0;
        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        g_foc.ctrl.iq.integral = g_foc.data.Vq;
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

    g_foc.startup.theta += (g_foc.startup.omega * CONTROL_PERIOD) / PI;
    g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);
    g_foc.data.theta_elec = g_foc.startup.theta;

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);
    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    g_foc.cmd.Id_ref = 0.0f;
    g_foc.cmd.Iq_ref = g_foc.cfg.startup_current;

    /* Calculate startup voltage directly (open-loop) */
    float v_resistive = g_foc.cmd.Iq_ref * g_foc.cfg.motor_rs;
    float v_bemf = g_foc.startup.omega * g_foc.cfg.motor_flux; /* Back-EMF term */
    float startup_voltage = v_resistive + v_bemf;

    /* Ensure minimum voltage to overcome friction and start rotation */
    float min_voltage = -g_foc.data.Vbus / SQRT3; /* Minimum voltage [V] */
    if (startup_voltage < min_voltage) {
        startup_voltage = min_voltage;
    }

    /* Clamp to safe maximum */
    float max_voltage = g_foc.data.Vbus / SQRT3;
    if (startup_voltage > max_voltage) {
        startup_voltage = max_voltage;
    }

    /* Apply voltage in q-axis only (for torque), d-axis = 0 */
    g_foc.data.Vd = 0.0f;
    g_foc.data.Vq = startup_voltage;

    /* Inverse Park */
    inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, cos_th, sin_th, &g_foc.data.Valpha,
                           &g_foc.data.Vbeta);

    svpwm_calculate();

    SMO_Update(&g_foc.ctrl.smo, g_foc.data.Valpha, g_foc.data.Vbeta, g_foc.data.Ialpha,
               g_foc.data.Ibeta);

#if ENABLE_CLOSED_LOOP_HANDOFF
    if (g_foc.startup.omega >= handoff_omega * 0.9f) {
        g_foc.ctrl.smo.pll_integral = g_foc.startup.omega;
        g_foc.ctrl.smo.omega_est = g_foc.startup.omega;

        g_foc.ctrl.iq.integral = g_foc.data.Vq;
        g_foc.ctrl.id.integral = g_foc.data.Vd;

        s_Id_filt = g_foc.data.Id;
        s_Iq_filt = g_foc.data.Iq;

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = g_foc.startup.omega;
            g_foc.ctrl.speed.integral = g_foc.cmd.Iq_ref;
        } else {
            g_foc.cmd.Iq_ref = g_foc.data.Iq;
        }
        s_blend_alpha = 0.0f;
        s_transition_counter = 0;
        s_transition_samples = (uint32_t)(TRANSITION_BLEND_MS * 0.001f * (float)CONTROL_FREQUENCY);
        if (s_transition_samples < 1) s_transition_samples = 1;
        s_in_transition = 1;
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
    /* Gate the 6th harmonic compensator to start 200 ms after entering closed-loop */
    s_closed_loop_counter++;
    uint32_t gate_samples = (uint32_t)(0.2f * (float)CONTROL_FREQUENCY);
    if (s_closed_loop_counter >= gate_samples) {
        g_foc.ctrl.smo.enable_harmonic_comp = 1;
    } else {
        g_foc.ctrl.smo.enable_harmonic_comp = 0;
    }

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
    s_Id_filt += g_foc.cfg.dq_filt_a * (g_foc.data.Id - s_Id_filt);
    s_Iq_filt += g_foc.cfg.dq_filt_a * (g_foc.data.Iq - s_Iq_filt);

    /* Cross-coupling compensation for rotating-frame LPF phase lag. */
    float tau = CONTROL_PERIOD * (1.0f / g_foc.cfg.dq_filt_a - 1.0f);
    float omega_tau = g_foc.data.omega_elec * tau;
    g_foc.data.Id = s_Id_filt + omega_tau * s_Iq_filt;
    g_foc.data.Iq = s_Iq_filt - omega_tau * s_Id_filt;

    /*=======================================================================*/
    /* Speed / Torque Control                                                */
    /*=======================================================================*/
    if (g_foc.status.control_mode == FOC_MODE_SPEED) {
        float accel_rate =
            g_foc.cfg.speed_ramp_accel * RPM_TO_RAD * g_foc.cfg.motor_poles * CONTROL_PERIOD;
        float decel_rate =
            g_foc.cfg.speed_ramp_decel * RPM_TO_RAD * g_foc.cfg.motor_poles * CONTROL_PERIOD;

        float ramp_error = g_foc.cmd.speed_ref_target - g_foc.cmd.speed_ref;

        if (ramp_error > accel_rate) {
            g_foc.cmd.speed_ref += accel_rate;
        } else if (ramp_error < -decel_rate) {
            g_foc.cmd.speed_ref -= decel_rate;
        } else {
            g_foc.cmd.speed_ref = g_foc.cmd.speed_ref_target;
        }

        float speed_error = g_foc.cmd.speed_ref - g_foc.data.omega_elec;
        g_foc.cmd.Iq_ref = PI_Update(&g_foc.ctrl.speed, speed_error);
    } else if (g_foc.status.control_mode == FOC_MODE_TORQUE) {
        float current_ramp_step = g_foc.cfg.current_ramp_rate * CONTROL_PERIOD;
        float ramp_error = g_foc.cmd.Iq_ref_target - g_foc.cmd.Iq_ref;
        if (ramp_error > current_ramp_step) {
            g_foc.cmd.Iq_ref += current_ramp_step;
        } else if (ramp_error < -current_ramp_step) {
            g_foc.cmd.Iq_ref -= current_ramp_step;
        } else {
            g_foc.cmd.Iq_ref = g_foc.cmd.Iq_ref_target;
        }
    }

    if (g_foc.ctrl.bist.mode != BIST_MODE_OFF) {
        BIST_Update(&g_foc.ctrl.bist, (float*)&g_foc.cmd.Iq_ref);
    }

    g_foc.cmd.Id_ref = 0.0f;

    if (g_foc.cmd.Iq_ref > g_foc.cfg.motor_max_curr) {
        g_foc.cmd.Iq_ref = g_foc.cfg.motor_max_curr;
    } else if (g_foc.cmd.Iq_ref < -g_foc.cfg.motor_max_curr) {
        g_foc.cmd.Iq_ref = -g_foc.cfg.motor_max_curr;
    }

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    float ff_gain = 0.5f;
    float omega_Ls = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_ls;
    float E_bemf = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_flux;

    g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq;
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + omega_Ls * g_foc.data.Id + E_bemf;

    /* Compute advanced angle for voltage command (Inverse Park) to compensate for PWM delay */
    float theta_adv = g_foc.data.theta_elec +
                      g_foc.data.omega_elec * (g_foc.cfg.comp_delay_samples * CONTROL_PERIOD) / PI;
    theta_adv = normalize_angle_norm(theta_adv);
    float cos_th_adv, sin_th_adv;
    cordic_sincos(theta_adv, &cos_th_adv, &sin_th_adv);

    inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, cos_th_adv, sin_th_adv, &g_foc.data.Valpha,
                           &g_foc.data.Vbeta);
    svpwm_calculate();

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
            g_foc.cmd.speed_ref = 0.0f;
            PI_Reset(&g_foc.ctrl.id);
            PI_Reset(&g_foc.ctrl.iq);
            PI_Reset(&g_foc.ctrl.speed);
            s_stopping = 0;
            s_stop_counter = 0;
            g_foc.cmd.speed_ref_target = s_saved_speed_target;
            g_foc.cmd.Iq_ref_target = s_saved_iq_target;
            g_foc.status.state = FOC_STATE_IDLE;
        }
    }
}

static void FOC_StateStop(void) {
    FOC_EnableDrivers(0);
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
    if (g_foc.status.state != FOC_STATE_FAULT) {
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

static void FOC_StateFault(void) {
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
    inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, cos_th, sin_th, &g_foc.data.Valpha,
                           &g_foc.data.Vbeta);
    svpwm_calculate();

    if (id_result.state == MOTOR_ID_STATE_COMPLETE) {
        MotorID_Stop();
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
        g_foc.status.fault = FOC_FAULT_NONE;
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

void FOC_EnableDrivers(uint8_t enable) {
    if (enable) {
        /* Set SD pins HIGH to enable IR2184S */
        LL_GPIO_SetOutputPin(SD1_GPIO_Port, SD1_Pin);
        LL_GPIO_SetOutputPin(SD2_GPIO_Port, SD2_Pin);
        LL_GPIO_SetOutputPin(SD3_GPIO_Port, SD3_Pin);
    } else {
        /* Set SD pins LOW to disable IR2184S */
        LL_GPIO_ResetOutputPin(SD1_GPIO_Port, SD1_Pin);
        LL_GPIO_ResetOutputPin(SD2_GPIO_Port, SD2_Pin);
        LL_GPIO_ResetOutputPin(SD3_GPIO_Port, SD3_Pin);
    }
}

void FOC_EnableDriver(uint8_t phase, uint8_t enable) {
    switch (phase) {
        case 1:
            if (enable) {
                LL_GPIO_SetOutputPin(SD1_GPIO_Port, SD1_Pin);
            } else {
                LL_GPIO_ResetOutputPin(SD1_GPIO_Port, SD1_Pin);
            }
            break;
        case 2:
            if (enable) {
                LL_GPIO_SetOutputPin(SD2_GPIO_Port, SD2_Pin);
            } else {
                LL_GPIO_ResetOutputPin(SD2_GPIO_Port, SD2_Pin);
            }
            break;
        case 3:
            if (enable) {
                LL_GPIO_SetOutputPin(SD3_GPIO_Port, SD3_Pin);
            } else {
                LL_GPIO_ResetOutputPin(SD3_GPIO_Port, SD3_Pin);
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