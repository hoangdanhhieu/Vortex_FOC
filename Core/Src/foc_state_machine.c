/**
 * @file foc_state_machine.c
 * @brief FOC State Machine and Main Control Implementation
 */

#include "foc_state_machine.h"

#include "comm_protocol.h"
#include "foc.h"
#include "foc_config.h"
#include "main.h"
#include "motor_id.h"
#include "motor_params.h"
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

/* Static filter state for current measurements */
static float s_Ia_filt = 0.0f;
static float s_Ib_filt = 0.0f;
static float s_Ic_filt = 0.0f;

static float s_Id_filt = 0.0f;
static float s_Iq_filt = 0.0f;

/* Safety check state */
static uint8_t oc_count = 0;       /* Overcurrent deglitch counter */
static uint32_t stall_counter = 0; /* Stall timer (ISR ticks) */

/* Transition blending state (open-loop → closed-loop) */
static float s_blend_alpha = 0.0f; /* 0 = open-loop, 1 = closed-loop */
static uint32_t s_transition_counter = 0;
static uint32_t s_transition_samples = 0;
static uint8_t s_in_transition = 0;

/* reverse is now g_foc.reverse */

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

static void FOC_StateIdle(void);
static void FOC_StateCalibration(void);
static void FOC_StateAlign(void);
static void FOC_StateStartup(void);
static void FOC_StateRun(void);
static void FOC_StateStop(void);
static void FOC_StateFault(void);
static void FOC_StateSelfCommission(void);
static void FOC_ApplyCurrentFilter(float* Ia, float* Ib, float* Ic);
void FOC_ResetCurrentFilter(void);

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
    g_foc.data.Vbus = MOTOR_VBUS_NOMINAL;

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
    PI_Init(&g_foc.ctrl.id, PI_ID_KP, PI_ID_KI, -MOTOR_VBUS_NOMINAL / SQRT3,
            MOTOR_VBUS_NOMINAL / SQRT3, CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.id, -MOTOR_VBUS_NOMINAL / SQRT3, MOTOR_VBUS_NOMINAL / SQRT3);

    PI_Init(&g_foc.ctrl.iq, PI_IQ_KP, PI_IQ_KI, -MOTOR_VBUS_NOMINAL / SQRT3,
            MOTOR_VBUS_NOMINAL / SQRT3, CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.iq, -MOTOR_VBUS_NOMINAL / SQRT3, MOTOR_VBUS_NOMINAL / SQRT3);

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
    g_foc.adc_cal.cal_samples = 0;

    /* Statistics */
    g_foc.status.run_counter = 0;
    g_foc.isr_time_cycles = 0;

    /* Disable drivers initially */
    FOC_EnableDrivers(0);

    /* Initialize Motor ID */
    MotorID_Init();

    /* Reset current filter */
    FOC_ResetCurrentFilter();
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

        /* Reset current filter */
        FOC_ResetCurrentFilter();

        /* Reset transition state */
        s_blend_alpha = 0.0f;
        s_transition_counter = 0;
        s_in_transition = 0;

        /* Enter calibration state */
        g_foc.adc_cal.offset_a = 0;
        g_foc.adc_cal.offset_b = 0;
        g_foc.adc_cal.offset_c = 0;
        g_foc.adc_cal.cal_samples = 0;
        g_foc.status.state = FOC_STATE_CALIBRATION;
    }
}

void FOC_Stop(void) {
    if (g_foc.status.state == FOC_STATE_RUN || g_foc.status.state == FOC_STATE_STARTUP) {
        g_foc.status.state = FOC_STATE_STOP;
    }
}

CCMRAM_FUNC static void FOC_CheckSafety(void) {
    /* Skip safety during calibration and idle */
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_CALIBRATION ||
        g_foc.status.state == FOC_STATE_FAULT || g_foc.status.state == FOC_STATE_SELF_COMMISSION) {
        oc_count = 0;
        stall_counter = 0;
        return;
    }

    /*--- Overcurrent (deglitched) ---*/
    float abs_ia = (g_foc.data.Ia >= 0.0f) ? g_foc.data.Ia : -g_foc.data.Ia;
    float abs_ib = (g_foc.data.Ib >= 0.0f) ? g_foc.data.Ib : -g_foc.data.Ib;
    float abs_ic = (g_foc.data.Ic >= 0.0f) ? g_foc.data.Ic : -g_foc.data.Ic;

    if (abs_ia > g_foc.cfg.fault_oc_threshold || abs_ib > g_foc.cfg.fault_oc_threshold ||
        abs_ic > g_foc.cfg.fault_oc_threshold) {
        if (++oc_count >= g_foc.cfg.fault_oc_count) {
            g_foc.status.fault = FOC_FAULT_OVERCURRENT;
            g_foc.status.state = FOC_STATE_FAULT;
        }
    } else {
        if (oc_count > 0) oc_count--;
    }

    /*--- Bus Voltage protection ---*/
    if (g_foc.data.Vbus > g_foc.cfg.fault_ov_threshold) {
        g_foc.status.fault = FOC_FAULT_OVERVOLTAGE;
        g_foc.status.state = FOC_STATE_FAULT;
    } else if (g_foc.data.Vbus < g_foc.cfg.fault_uv_threshold) {
        g_foc.status.fault = FOC_FAULT_UNDERVOLTAGE;
        g_foc.status.state = FOC_STATE_FAULT;
    }

    /*--- Stall detection ---*/
    if (g_foc.cfg.fault_stall_enable && g_foc.status.state == FOC_STATE_RUN) {
        float abs_speed =
            (g_foc.data.speed_rpm >= 0.0f) ? g_foc.data.speed_rpm : -g_foc.data.speed_rpm;
        float abs_iq = (g_foc.data.Iq >= 0.0f) ? g_foc.data.Iq : -g_foc.data.Iq;

        if (abs_speed < g_foc.cfg.fault_stall_speed && abs_iq > g_foc.cfg.fault_stall_current) {
            stall_counter++;
            if (stall_counter >
                (uint32_t)(g_foc.cfg.fault_stall_time_ms / (CONTROL_PERIOD * 1000.0f))) {
                g_foc.status.fault = FOC_FAULT_STALL;
                g_foc.status.state = FOC_STATE_FAULT;
            }
        } else {
            /* Fast recovery if it was just a noise transient */
            if (stall_counter > 0) stall_counter--;
        }
    } else if (g_foc.status.state != FOC_STATE_RUN) {
        stall_counter = 0;
    }
}

/*===========================================================================*/
/* Main ISR Entry Point                                                      */
/*===========================================================================*/
CCMRAM_FUNC void FOC_HighFrequencyTask(uint16_t adc_ia, uint16_t adc_ib, uint16_t adc_ic,
                                       uint16_t adc_vbus) {
    if (!foc_initialized) return;
    g_foc.data.Vbus = foc_adc_to_vbus(adc_vbus);
    if (g_foc.data.Vbus < 1.0f) g_foc.data.Vbus = 1.0f; /* Safety clamp */
    g_foc.data.Vbus_inv = 1.0f / g_foc.data.Vbus;       /* Precompute once */
    g_foc.data.Ia = ((float)adc_ia - (float)g_foc.adc_cal.offset_a) * ADC_TO_CURRENT;
    g_foc.data.Ib = ((float)adc_ib - (float)g_foc.adc_cal.offset_b) * ADC_TO_CURRENT;
    g_foc.data.Ic = ((float)adc_ic - (float)g_foc.adc_cal.offset_c) * ADC_TO_CURRENT;

    g_foc.status.run_counter++;

    FOC_CheckSafety();

    if (g_foc.status.reverse < 0.0f) {
        float t = g_foc.data.Ib;
        g_foc.data.Ib = g_foc.data.Ic;
        g_foc.data.Ic = t;
    }

    foc_reconstruct_currents(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Ic, g_foc.data.duty_a,
                             g_foc.data.duty_b, g_foc.data.duty_c, &g_foc.data.Ia, &g_foc.data.Ib,
                             &g_foc.data.Ic);

    FOC_ApplyCurrentFilter(&g_foc.data.Ia, &g_foc.data.Ib, &g_foc.data.Ic);

    clarke_transform(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Ic, &g_foc.data.Ialpha,
                     &g_foc.data.Ibeta);

    switch (g_foc.status.state) {
        case FOC_STATE_IDLE:
            FOC_StateIdle();
            break;
        case FOC_STATE_CALIBRATION:
            g_foc.adc_cal.offset_a += adc_ia;
            g_foc.adc_cal.offset_b += adc_ib;
            g_foc.adc_cal.offset_c += adc_ic;
            g_foc.adc_cal.cal_samples++;
            FOC_StateCalibration();
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
            FOC_StateStop();
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
            g_foc.plot.ready = 1;
        }
    }

    /* Apply duty cycles to PWM */
    if (g_foc.status.reverse > 0) {
        foc_set_pwm_duty(g_foc.data.duty_a, g_foc.data.duty_b, g_foc.data.duty_c);
    } else {
        foc_set_pwm_duty(g_foc.data.duty_a, g_foc.data.duty_c, g_foc.data.duty_b);
    }
}

/*===========================================================================*/
/* Current Filter Implementation                                             */
/*===========================================================================*/

/**
 * @brief Apply IIR 1st order lowpass filter to current measurements
 * @param Ia Pointer to phase A current (in/out)
 * @param Ib Pointer to phase B current (in/out)
 * @param Ic Pointer to phase C current (in/out)
 * @note Filter equation: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 */
CCMRAM_FUNC static void FOC_ApplyCurrentFilter(float* Ia, float* Ib, float* Ic) {
    /* IIR 1st order lowpass filter */
    s_Ia_filt += g_foc.cfg.adc_filt_a * (*Ia - s_Ia_filt);
    s_Ib_filt += g_foc.cfg.adc_filt_a * (*Ib - s_Ib_filt);
    s_Ic_filt += g_foc.cfg.adc_filt_a * (*Ic - s_Ic_filt);

    /* Output filtered values */
    *Ia = s_Ia_filt;
    *Ib = s_Ib_filt;
    *Ic = s_Ic_filt;
}

/**
 * @brief Reset current filter states (call during calibration)
 */
void FOC_ResetCurrentFilter(void) {
    s_Ia_filt = 0.0f;
    s_Ib_filt = 0.0f;
    s_Ic_filt = 0.0f;
}

/*===========================================================================*/
/* State Implementations                                                     */
/*===========================================================================*/

static void FOC_StateIdle(void) {
    /* Keep PWM at 50% (all low-side off with IR2184S) */
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
    FOC_EnableDrivers(0);
}

static void FOC_StateCalibration(void) {
    /* Continue accumulating for CAL_SAMPLES */
    if (g_foc.adc_cal.cal_samples >= CAL_SAMPLES) {
        /* Calculate average offsets */
        g_foc.adc_cal.offset_a /= CAL_SAMPLES;
        g_foc.adc_cal.offset_b /= CAL_SAMPLES;
        g_foc.adc_cal.offset_c /= CAL_SAMPLES;

        /* Initialize startup */
        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        g_foc.startup.counter = 0;

        if (id_result.state != MOTOR_ID_STATE_ALIGN) {
            FOC_EnableDrivers(1);
            g_foc.status.state = FOC_STATE_ALIGN;
        } else {
            g_foc.status.state = FOC_STATE_SELF_COMMISSION;
        }
    }
}

static void FOC_StateAlign(void) {
    g_foc.startup.counter++;

    /* Force angle to 0 */
    g_foc.data.theta_elec = 0.0f;
    g_foc.data.omega_elec = 0.0f;
    g_foc.data.speed_rpm = 0.0f;

    /* Park transform */
    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, g_foc.data.theta_elec, &g_foc.data.Id,
                   &g_foc.data.Iq);

    /* Current control to fixed vector (Id=ALIGN_CURRENT, Iq=0) */
    g_foc.cmd.Id_ref = g_foc.cfg.align_current;
    g_foc.cmd.Iq_ref = 0.0f;

    /* PI Controllers */
    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error);
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error);

    /* Inverse Park */
    inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, g_foc.data.theta_elec, &g_foc.data.Valpha,
                           &g_foc.data.Vbeta);

    /* SVPWM with integrated dead-time compensation */
    svpwm_calculate(&g_foc.data.Valpha, &g_foc.data.Vbeta, g_foc.data.Vbus, g_foc.data.Ia,
                    g_foc.data.Ib, g_foc.data.Ic, &g_foc.data.duty_a, &g_foc.data.duty_b,
                    &g_foc.data.duty_c);

    /* Transition to startup after duration */
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

CCMRAM_FUNC static void FOC_StateStartup(void) {
    g_foc.startup.counter++;

    /* Open-loop acceleration */
    float accel_rad = g_foc.cfg.startup_accel * RPM_TO_RAD * g_foc.cfg.motor_poles;
    g_foc.startup.omega += accel_rad * CONTROL_PERIOD;

    /* Clamp startup speed */
    float handoff_omega = g_foc.cfg.startup_handoff_speed * RPM_TO_RAD * g_foc.cfg.motor_poles;
    if (g_foc.startup.omega > handoff_omega) {
        g_foc.startup.omega = handoff_omega;
    }

    g_foc.startup.theta += (g_foc.startup.omega * CONTROL_PERIOD) / PI;
    g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);
    g_foc.data.theta_elec = g_foc.startup.theta;

    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, g_foc.data.theta_elec, &g_foc.data.Id,
                   &g_foc.data.Iq);

    /* Open-loop voltage control for startup */
    g_foc.cmd.Id_ref = 0.0f;
    g_foc.cmd.Iq_ref = g_foc.cfg.startup_current;

    /* Calculate startup voltage directly (open-loop) */
    float v_resistive = g_foc.cmd.Iq_ref * g_foc.cfg.motor_rs;
    float v_bemf = g_foc.startup.omega * g_foc.cfg.motor_flux; /* Back-EMF term */
    float startup_voltage = v_resistive + v_bemf + 0.3f;

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
    inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, g_foc.data.theta_elec, &g_foc.data.Valpha,
                           &g_foc.data.Vbeta);

    svpwm_calculate(&g_foc.data.Valpha, &g_foc.data.Vbeta, g_foc.data.Vbus, g_foc.data.Ia,
                    g_foc.data.Ib, g_foc.data.Ic, &g_foc.data.duty_a, &g_foc.data.duty_b,
                    &g_foc.data.duty_c);

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
    float timeout_samples = (float)STARTUP_TIMEOUT_MS * 0.001f * (float)CONTROL_FREQUENCY;
    if (g_foc.startup.counter > (uint32_t)timeout_samples) {
        g_foc.status.fault = FOC_FAULT_STARTUP_FAIL;
        g_foc.status.state = FOC_STATE_FAULT;
    }
#endif
}

CCMRAM_FUNC static void FOC_StateRun(void) {
    /* Update SMO observer */
    SMO_Update(&g_foc.ctrl.smo, g_foc.data.Valpha, g_foc.data.Vbeta, g_foc.data.Ialpha,
               g_foc.data.Ibeta);

    /* Get SMO estimates */
    float smo_theta = SMO_GetAngle(&g_foc.ctrl.smo);
    float smo_omega = SMO_GetSpeed(&g_foc.ctrl.smo);
    float smo_speed_rpm = SMO_GetSpeedRPM(&g_foc.ctrl.smo);

    if (s_in_transition) {
        /* Linear blend from open-loop to SMO angle */
        s_transition_counter++;
        s_blend_alpha = (float)s_transition_counter / (float)s_transition_samples;
        if (s_blend_alpha >= 1.0f) {
            s_blend_alpha = 1.0f;
            s_in_transition = 0;
        }

        /* Continue open-loop angle integration during transition */
        g_foc.startup.theta += (g_foc.startup.omega * CONTROL_PERIOD) / PI;
        g_foc.startup.theta = normalize_angle_norm(g_foc.startup.theta);

        /* Blend angles: lerp with wrap-around handling */
        float delta = normalize_angle_norm(smo_theta - g_foc.startup.theta);

        g_foc.data.theta_elec = normalize_angle_norm(g_foc.startup.theta + s_blend_alpha * delta);

        /* Blend speed similarly */
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

    /* Computational delay compensation */
    g_foc.data.theta_elec +=
        g_foc.data.omega_elec * (g_foc.cfg.comp_delay_samples * CONTROL_PERIOD) / PI;
    g_foc.data.theta_elec = normalize_angle_norm(g_foc.data.theta_elec);

    /* Park transform */
    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, g_foc.data.theta_elec, &g_foc.data.Id,
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

    /* Process BIST 8kHz Profiler Override */
    if (g_foc.ctrl.bist.mode != BIST_MODE_OFF) {
        BIST_Update(&g_foc.ctrl.bist, (float*)&g_foc.cmd.Iq_ref);
    }

    /* D-axis: always regulate to 0 (or field weakening) */
    g_foc.cmd.Id_ref = 0.0f;

    /* Current limit */
    if (g_foc.cmd.Iq_ref > g_foc.cfg.motor_max_curr) {
        g_foc.cmd.Iq_ref = g_foc.cfg.motor_max_curr;
    } else if (g_foc.cmd.Iq_ref < -g_foc.cfg.motor_max_curr) {
        g_foc.cmd.Iq_ref = -g_foc.cfg.motor_max_curr;
    }

    /* Current PI controllers */
    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;

    float ff_gain = 0.5f;
    float omega_Ls = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_ls;
    float E_bemf = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_flux;

    g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq;
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + omega_Ls * g_foc.data.Id + E_bemf;

    /* Inverse Park */
    inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, g_foc.data.theta_elec, &g_foc.data.Valpha,
                           &g_foc.data.Vbeta);

    /* SVPWM with integrated dead-time compensation */
    svpwm_calculate(&g_foc.data.Valpha, &g_foc.data.Vbeta, g_foc.data.Vbus, g_foc.data.Ia,
                    g_foc.data.Ib, g_foc.data.Ic, &g_foc.data.duty_a, &g_foc.data.duty_b,
                    &g_foc.data.duty_c);
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
    MotorID_RunStep(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Vbus, &g_foc.data.duty_a,
                    &g_foc.data.duty_b, &g_foc.data.duty_c);
    if (id_result.state == MOTOR_ID_STATE_COMPLETE) {
        MotorID_Stop();
        FOC_StateStop();
    }
}

/*===========================================================================*/
/* API Functions                                                             */
/*===========================================================================*/

void FOC_SetSpeedRef(float speed_rpm) {
    g_foc.cmd.speed_ref_target = speed_rpm * RPM_TO_RAD * g_foc.cfg.motor_poles;
}

void FOC_SetTorqueRef(float torque_percent) {
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
        MotorID_Start();
        FOC_EnableDrivers(0);
        g_foc.status.state = FOC_STATE_SELF_COMMISSION;
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
