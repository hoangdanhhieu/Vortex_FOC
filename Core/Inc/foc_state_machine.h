/**
 * @file foc_state_machine.h
 * @brief FOC State Machine and Main Control Structure
 */

#ifndef FOC_STATE_MACHINE_H
#define FOC_STATE_MACHINE_H

#include <stdint.h>

#include "flash_config.h"
#include "foc_beep.h"
#include "ladrc_controller.h"
#include "pi_controller.h"
#include "response_profiler.h"
#include "smo_observer.h"

/*===========================================================================*/
/* FOC State Definitions                                                     */
/*===========================================================================*/

typedef enum {
    FOC_STATE_IDLE = 0,        /**< Motor stopped, waiting for start command */
    FOC_STATE_CALIBRATION,     /**< ADC offset calibration */
    FOC_STATE_DETECT,          /**< BEMF detection (flying start or normal) */
    FOC_STATE_FLYING_START,    /**< PLL locking on BEMF for flying start */
    FOC_STATE_ALIGN,           /**< Rotor alignment */
    FOC_STATE_STARTUP,         /**< Open-loop ramp-up */
    FOC_STATE_RUN,             /**< Closed-loop FOC running */
    FOC_STATE_ZERO_CROSS,      /**< Seamless 4-quadrant zero-crossing virtual commutation bridge */
    FOC_STATE_STOP,            /**< Controlled stop */
    FOC_STATE_FAULT,           /**< Fault condition */
    FOC_STATE_SELF_COMMISSION, /**< Motor parameter identification */
    FOC_STATE_COAST_FLUX_ID,   /**< Freewheeling BEMF measurement */
    FOC_STATE_BEEP             /**< Power-on beep sequence (ESC-style) */
} FOC_State_t;

typedef enum {
    FOC_MODE_SPEED = 0, /**< Speed control mode */
    FOC_MODE_TORQUE,    /**< Torque (current) control mode */
    FOC_MODE_VOLTAGE    /**< Voltage control mode */
} FOC_ControlMode_t;

typedef enum {
    FOC_FAULT_NONE = 0,
    FOC_FAULT_OVERCURRENT,
    FOC_FAULT_OVERVOLTAGE,
    FOC_FAULT_UNDERVOLTAGE,
    FOC_FAULT_STARTUP_FAIL,
    FOC_FAULT_OBSERVER_FAIL,
    FOC_FAULT_STALL,
    FOC_FAULT_GROUND
} FOC_Fault_t;

/*===========================================================================*/
/* FOC Control Structure                                                     */
/*===========================================================================*/

typedef struct {
    FOC_State_t state;              /**< Current state machine state */
    FOC_ControlMode_t control_mode; /**< Control mode: Speed, Torque (Current), or Voltage */
    FOC_Fault_t fault;              /**< Active system fault code */
    float reverse;         /**< Motor rotation direction [dimensionless: 1.0 = FWD, -1.0 = REV] */
    uint8_t in_transition; /**< Open-loop to closed-loop handoff flag [0 = Normal, 1 = Blending] */
    uint32_t run_counter;  /**< Control loop execution tick counter */
} FOC_Status_t;

typedef struct {
    float Ia, Ib, Ic;            /**< 3-phase raw measured currents [A] */
    float Ialpha, Ibeta;         /**< Clarke stationary frame currents (alpha, beta) [A] */
    float Ialpha_flt, Ibeta_flt; /**< STF (Self-Tuning Filter) cleaned currents (alpha, beta) [A] */
    float Vphase_a, Vphase_b, Vphase_c; /**< 3-phase terminal voltages [V] */
    float Id, Iq;                       /**< Park rotating frame currents (d-axis, q-axis) [A] */
    float Vd, Vq;        /**< Park rotating frame control voltages (d-axis, q-axis) [V] */
    float Iq_ref_cmd;    /**< Slew-rate limited active Iq current command [A] */
    float Valpha, Vbeta; /**< Inverse Park stationary frame output voltages [V] */
    float Valpha_applied,
        Vbeta_applied; /**< Voltages physically applied to motor during previous PWM period [V] */
    float theta_park;  /**< Electrical rotor position angle for Park transform (no hardware delay
             compensation) */
    float theta_elec;  /**< Electrical rotor position angle for PWM [normalized: -1.0 to 1.0,
             where 1.0 = +pi rad] */
    float omega_elec;  /**< Electrical angular velocity [rad/s] */
    float Vbus;        /**< DC bus supply voltage [V] */
    float Vbus_inv;    /**< Inverse of DC bus voltage (1.0 / Vbus) [1/V] */
    float Ibus;        /**< Total DC bus consumed current [A] */
    float duty_a, duty_b, duty_c; /**< Inverter phase PWM duty cycles [dimensionless: 0.0 to 1.0] */
    float i_scale;                /**< ADC raw count to phase current conversion factor [A/count] */
    float v_scale;                /**< ADC raw count to phase voltage conversion factor [V/count] */
    float inv_i_th;               /**< Precomputed inverse deadtime threshold [1/A] */
    float stall_risk;             /**< Real-time leaky stall risk accumulator [0.0 to 1.0] */
    float eta_em;                 /**< Scale-free electromechanical power conversion ratio */
    float d_desync;               /**< Back-EMF vector desynchronization index [0.0 to 1.0] */
    float r_bemf; /**< Residual Back-EMF ratio (Hallucination buster) [0.0 to 1.0] */
} FOC_Data_t;

typedef struct {
    float speed_ref;        /**< Ramp-filtered electrical speed target [rad/s] */
    float speed_ref_target; /**< Commanded electrical speed target from user/host [rad/s] */
    float Iq_ref;           /**< Ramp-filtered quadrature current target [A] */
    float Iq_ref_target;    /**< Commanded quadrature current target from user/host [A] */
    float Id_ref;           /**< Direct axis current reference (0A or field weakening) [A] */
    float Id_ref_target;    /**< Commanded direct axis current target [A] */
    float Vq_ref; /**< Ramp-filtered normalized voltage command [normalized: 0.0 to 1.0, 1.0 = SVPWM
                     circle limit] */
    float Vq_ref_target; /**< Commanded normalized voltage target [normalized: 0.0 to 1.0, 1.0 =
                            SVPWM circle limit] */
} FOC_Cmd_t;

typedef struct {
    float user_plot1; /**< Custom debug variable 1 to route to GUI */
    float user_plot2; /**< Custom debug variable 2 to route to GUI */
    float user_plot3; /**< Custom debug variable 3 to route to GUI */
} FOC_Plot_t;

typedef struct {
    float bridge_dir;          /**< Virtual commutation sweep direction (+1.0 or -1.0) */
    float bridge_omega;        /**< Virtual electrical speed (rad/s) */
    float bridge_theta;        /**< Virtual electrical angle [normalized: -1.0 to 1.0] */
    float bridge_target_omega; /**< Target handoff speed (rad/s) */
    float bridge_slew_rate;    /**< Dynamic acceleration of virtual bridge (rad/s^2) */
    float id_current;          /**< Ramped d-axis current for soft magnetic well entry (A) */
    float iq_current;          /**< Ramped q-axis current in bridge (A) */
    uint16_t debounce_cnt;     /**< Debounce counter for zero-cross entry (samples) */
    uint16_t lockout_cnt;      /**< Lockout timer preventing rapid re-triggering (samples) */
    uint16_t grace_counter;    /**< Observer settling grace counter post-handoff (samples) */
    uint8_t was_reversing;     /**< Voltage-mode reversal edge tracking flag */
} FOC_ZeroCross_t;

/*===========================================================================*/
/* FOC Control Structure                                                     */
/*===========================================================================*/

typedef struct {
    /*--- Main State ---*/
    FOC_Status_t status;

    /*--- Controllers ---*/
    struct {
        PI_Controller_t id;       /**< Direct axis (d-axis) current PI controller [A -> V] */
        PI_Controller_t iq;       /**< Quadrature axis (q-axis) current PI controller [A -> V] */
        LADRC_Controller_t speed; /**< Speed loop Linear ADRC controller [rad/s elec -> A] */
        SMO_Observer_t smo; /**< Sensorless Sliding Mode Observer for angle/speed estimation */
        union {
            Profiler_State_t profiler; /**< Dynamic response profiler & test signal generator */
            Profiler_State_t bist;     /**< Backward-compatibility alias */
        };
    } ctrl;

    /*--- Live Data / Signals ---*/
    FOC_Data_t data;

    /*--- References / Commands ---*/
    FOC_Cmd_t cmd;

    /*--- Startup State ---*/
    struct {
        float theta;      /**< Open-loop electrical angle ramp [normalized: -1.0 to 1.0] */
        float omega;      /**< Open-loop electrical angular velocity [rad/s] */
        uint32_t counter; /**< Startup stage step/timer counter [ISR samples] */
    } startup;

    /*--- Zero-Crossing State (Internal FSM Context) ---*/
    FOC_ZeroCross_t zc; /**< Seamless 4-quadrant zero-crossing bridge context */

    /*--- Configuration (Directly from Flash) ---*/
    FlashConfig_t cfg; /**< Persistent motor and control parameters loaded from Flash */

    /*--- Dynamic Timing & Hardware Timer ---*/
    uint32_t timer_arr;   /**< TIM1 Auto-Reload value based on pwm_frequency */
    float timer_arr_f;    /**< Float copy of timer_arr for fast duty cycle calculation */
    float dt;             /**< Control loop sample period [s] (1.0 / pwm_frequency) */
    float deadtime_duty;  /**< Dynamic deadtime duty compensation [dimensionless] */
    float wc_current_stf; /**< Precomputed cutoff frequency for current STF filter [rad/s] */

    /*--- Constraints & Performance ---*/
    float max_duty; /**< Maximum allowed PWM duty cycle clamp [dimensionless: 0.0 to 1.0] */
    uint32_t isr_time_cycles; /**< ADC ISR execution time [CPU clock cycles @ 170MHz] */

    /*--- Plotting (Telemetry Snapshot) ---*/
    FOC_Plot_t plot;

} FOC_Control_t;

/*===========================================================================*/
/* Global FOC Instance                                                       */
/*===========================================================================*/

extern FOC_Control_t g_foc;

/*===========================================================================*/
/* Read-Only State & Telemetry Getters (Zero-Overhead Inline)                */
/*===========================================================================*/

static inline const FOC_Data_t* FOC_GetData(void) {
    return &g_foc.data;
}

/**
 * @brief Resolved overcurrent trip threshold [A].
 *
 *        fault_oc_threshold <= 0.001f -> 0.0f (Protection disabled / OFF)
 *        fault_oc_threshold  > 0.001f -> exact value in [0.1A, BOARD_MAX_CURRENT]
 *
 *        Single source of truth shared by the HW AWD configuration (FOC_ConfigureAWD).
 */
static inline float FOC_GetOCThreshold(void) {
    float thr = g_foc.cfg.fault_oc_threshold;
    if (thr <= 0.001f) {
        return 0.0f;
    }
    return clampf(thr, 0.1f, BOARD_MAX_CURRENT);
}

static inline const FOC_Status_t* FOC_GetStatus(void) {
    return &g_foc.status;
}

static inline FOC_Fault_t FOC_GetFault(void) {
    return g_foc.status.fault;
}

CCMRAM_FUNC static inline uint8_t FOC_IsInTransition(void) {
    return g_foc.status.in_transition;
}

static inline float FOC_GetSpeed(void) {
    return g_foc.data.omega_elec;
}

static inline float FOC_GetVbus(void) {
    return g_foc.data.Vbus;
}

static inline float FOC_GetId(void) {
    return g_foc.data.Id;
}

static inline float FOC_GetIq(void) {
    return g_foc.data.Iq;
}

static inline float FOC_GetVd(void) {
    return g_foc.data.Vd;
}

static inline float FOC_GetVq(void) {
    return g_foc.data.Vq;
}

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

/**
 * @brief Check if FOC system is initialized
 */
uint8_t FOC_IsInitialized(void);

/**
 * @brief Check if motor configuration parameters are valid for closed-loop operation.
 * @return 1 if fully configured and valid, 0 if unconfigured/invalid.
 */
uint8_t FOC_IsConfigValid(void);

/**
 * @brief Initialize FOC control structure
 */
void FOC_Init(void);

/**
 * @brief Start motor
 */
void FOC_Start(void);

/**
 * @brief Stop motor
 */
void FOC_Stop(void);

/**
 * @brief Main FOC control loop (call from ADC ISR)
 * @param adc1_data Raw ADC1 injected rank 1 value (phase depends on channel config)
 * @param adc2_data Raw ADC2 injected rank 1 value (phase depends on channel config)
 */
void FOC_HighFrequencyTask(uint16_t adc1_data, uint16_t adc2_data);

/**
 * @brief Set speed reference
 * @param speed_rad Target speed in electrical rad/s
 */
void FOC_SetSpeedRef(float speed_rad);

/**
 * @brief Set torque reference (Iq)
 * @param torque_percent Torque as percentage of max current (-100 to 100)
 */
void FOC_SetTorqueRef(float torque_percent);

/**
 * @brief Set torque reference directly in Amperes
 * @param iq_target_amp Target quadrature current [A]
 */
void FOC_SetTorqueCurrent(float iq_target_amp);

/**
 * @brief Set voltage reference (Vq)
 * @param voltage_percent Voltage as percentage of Vbus / sqrt(3) (-1 to 1)
 */
void FOC_SetVoltageRef(float voltage_percent);

/**
 * @brief Clear fault and return to IDLE
 */
void FOC_ClearFault(void);
void FOC_ResetStallDetector(void);
uint8_t FOC_IsReversalGraceActive(void);

void FOC_EnableDrivers(uint8_t enable);
void FOC_EnableDriver(uint8_t phase, uint8_t enable);

/**
 * @brief Start Motor Parameter Self-Commissioning
 */
void FOC_StartSelfCommission(void);

/**
 * @brief Check if the dynamic response profiler is currently active
 * @return 1 if profiler is running, 0 otherwise
 */
uint8_t FOC_IsProfilerActive(void);

/**
 * @brief Start a specific ESC beep sequence by type
 *
 * No-ops unless the motor is IDLE/STOP with no active fault. Excites the motor
 * with an impedance-compensated open-loop signal, then returns to IDLE. A start
 * command, fault, or sequence timeout aborts it immediately.
 *
 * @param type Sequence identifier from FOC_BeepType_t
 */
void FOC_PlayBeep(FOC_BeepType_t type);

#endif /* FOC_STATE_MACHINE_H */