/**
 * @file foc_state_machine.h
 * @brief FOC State Machine and Main Control Structure
 */

#ifndef FOC_STATE_MACHINE_H
#define FOC_STATE_MACHINE_H

#include <stdint.h>

#include "flash_config.h"
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
    FOC_STATE_STOP,            /**< Controlled stop */
    FOC_STATE_FAULT,           /**< Fault condition */
    FOC_STATE_SELF_COMMISSION, /**< Motor parameter identification */
    FOC_STATE_COAST_FLUX_ID,   /**< Freewheeling BEMF measurement */
    FOC_STATE_BRAKE            /**< Active dynamic braking for reverse flying start recovery */
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
    float theta_park;    /**< Electrical rotor position angle for Park transform (no hardware delay
                            compensation) */
    float theta_elec;    /**< Electrical rotor position angle for PWM [normalized: -1.0 to 1.0,
                            where 1.0 = +pi rad] */
    float omega_elec;    /**< Electrical angular velocity [rad/s] */
    float Vbus;          /**< DC bus supply voltage [V] */
    float Vbus_inv;      /**< Inverse of DC bus voltage (1.0 / Vbus) [1/V] */
    float Ibus;          /**< Total DC bus consumed current [A] */
    float duty_a, duty_b, duty_c; /**< Inverter phase PWM duty cycles [dimensionless: 0.0 to 1.0] */
    float i_scale;                /**< ADC raw count to phase current conversion factor [A/count] */
    float v_scale;                /**< ADC raw count to phase voltage conversion factor [V/count] */
    float e_real_flt;             /**< Filtered real Back-EMF vector magnitude [V] */
    float e_expect_flt;           /**< Filtered expected Back-EMF vector magnitude [V] */
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
    float Vq_ref;        /**< Ramp-filtered normalized voltage command [normalized: -1.0 to 1.0] */
    float Vq_ref_target; /**< Commanded normalized voltage target [normalized: -1.0 to 1.0] */
} FOC_Cmd_t;

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
        LADRC_Controller_t speed; /**< Speed loop Linear ADRC controller [RPM -> A] */
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

    /*--- Configuration (Directly from Flash) ---*/
    FlashConfig_t cfg; /**< Persistent motor and control parameters loaded from Flash */

    /*--- ADC Calibration & Offsets ---*/
    struct {
        int32_t offset_a; /**< Phase A current ADC zero-crossing offset [ADC counts: 0 to 4095] */
        int32_t offset_b; /**< Phase B current ADC zero-crossing offset [ADC counts: 0 to 4095] */
        int32_t offset_c_pb1; /**< Phase C current offset measured on ADC1 via PB1 [ADC counts: 0 to
                                 4095] */
        int32_t offset_c_opamp3; /**< Phase C current offset measured on ADC2 via VOPAMP3 [ADC
                                    counts: 0 to 4095] */
        int32_t offset_vphase_a; /**< Phase A voltage ADC zero-crossing offset [ADC counts: 0 to
                                    4095] */
        int32_t offset_vphase_b; /**< Phase B voltage ADC zero-crossing offset [ADC counts: 0 to
                                    4095] */
        int32_t offset_vphase_c; /**< Phase C voltage ADC zero-crossing offset [ADC counts: 0 to
                                    4095] */
        uint16_t cal_samples;    /**< Calibration sample accumulation count [samples] */
    } adc_cal;

    /*--- Board Noise Profile & Signal Integrity ---*/
    struct {
        float noise_rms;       /**< Current measurement RMS noise floor [A] */
        float noise_pk_pk;     /**< Current measurement Peak-to-Peak noise [A] */
        float is_flat_thr;     /**< Auto-adapted settling threshold for Motor ID [A] */
        float i_inj_min;       /**< Minimum AC injection current for SNR > 20dB [A] */
        float bemf_noise_sq;   /**< BEMF magnitude noise floor threshold [V^2] */
        uint8_t health_status; /**< Hardware health: 0=EXCELLENT, 1=GOOD, 2=NOISY, 3=FAULT */
    } noise_profile;

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
    struct {
        float user_plot1; /**< Custom debug variable 1 to route to GUI */
        float user_plot2; /**< Custom debug variable 2 to route to GUI */
        float user_plot3; /**< Custom debug variable 3 to route to GUI */
    } plot;

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

static inline const FOC_Status_t* FOC_GetStatus(void) {
    return &g_foc.status;
}

static inline FOC_State_t FOC_GetState(void) {
    return g_foc.status.state;
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
 * @param torque_percent Torque as percentage of max current (0 to 100)
 */
void FOC_SetTorqueRef(float torque_percent);

/**
 * @brief Set voltage reference (Vq)
 * @param voltage_percent Voltage as percentage of Vbus / sqrt(3) (0 to 100)
 */
void FOC_SetVoltageRef(float voltage_percent);

/**
 * @brief Set control mode
 * @param mode FOC_MODE_SPEED or FOC_MODE_TORQUE
 */
void FOC_SetControlMode(FOC_ControlMode_t mode);

/**
 * @brief Clear fault and return to IDLE
 */
void FOC_ClearFault(void);
void FOC_ResetStallDetector(void);

void FOC_EnableDrivers(uint8_t enable);
void FOC_EnableDriver(uint8_t phase, uint8_t enable);

/**
 * @brief Start Motor Parameter Self-Commissioning
 */
void FOC_StartSelfCommission(void);

/**
 * @brief Set motor rotation direction
 * @param dir  1 = forward (default), -1 = reverse
 * @note  Only effective when motor is IDLE; ignored while running.
 */
void FOC_SetDirection(int8_t dir);

/**
 * @brief Get current motor direction setting
 * @return 1 = forward, -1 = reverse
 */
int8_t FOC_GetDirection(void);

/**
 * @brief Get control loop sample period
 * @return Control loop sample period in seconds
 */
float FOC_GetDt(void);

/**
 * @brief Play a tune on motor phases
 */
void playTune(void);

#endif /* FOC_STATE_MACHINE_H */