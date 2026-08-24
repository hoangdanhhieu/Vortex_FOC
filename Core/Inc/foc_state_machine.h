/**
 * @file foc_state_machine.h
 * @brief FOC State Machine and Main Control Structure
 */

#ifndef FOC_STATE_MACHINE_H
#define FOC_STATE_MACHINE_H

#include <stdint.h>

#include "bist_profiler.h"
#include "flash_config.h"
#include "pi_controller.h"
#include "ladrc_controller.h"
#include "smo_observer.h"

/*===========================================================================*/
/* FOC State Definitions                                                     */
/*===========================================================================*/

typedef enum {
    FOC_STATE_IDLE = 0,       /**< Motor stopped, waiting for start command */
    FOC_STATE_CALIBRATION,    /**< ADC offset calibration */
    FOC_STATE_DETECT,         /**< BEMF detection (flying start or normal) */
    FOC_STATE_FLYING_START,   /**< PLL locking on BEMF for flying start */
    FOC_STATE_ALIGN,          /**< Rotor alignment */
    FOC_STATE_STARTUP,        /**< Open-loop ramp-up */
    FOC_STATE_RUN,            /**< Closed-loop FOC running */
    FOC_STATE_STOP,           /**< Controlled stop */
    FOC_STATE_FAULT,          /**< Fault condition */
    FOC_STATE_SELF_COMMISSION /**< Motor parameter identification */
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
    /*--- Main State ---*/
    struct {
        FOC_State_t state;
        FOC_ControlMode_t control_mode;
        FOC_Fault_t fault;
        float reverse; /**< 1.0 = FWD, -1.0 = REV */
        uint8_t in_transition; /**< 1 during open-loop to closed-loop handoff */
        uint32_t run_counter;
    } status;

    /*--- Controllers ---*/
    struct {
        PI_Controller_t id;
        PI_Controller_t iq;
        LADRC_Controller_t speed;
        SMO_Observer_t smo;
        BIST_State_t bist;
    } ctrl;

    /*--- Live Data / Signals ---*/
    struct {
        float Ia, Ib, Ic;
        float Ialpha, Ibeta;
        float Ialpha_flt, Ibeta_flt;
        float Vphase_a, Vphase_b, Vphase_c;
        float Id, Iq;
        float Vd, Vq;
        float Iq_ref_cmd;
        float Valpha, Vbeta;
        float theta_elec;
        float omega_elec;
        float speed_rpm;
        float Vbus;
        float Vbus_inv;
        float duty_a, duty_b, duty_c;
        float i_scale, v_scale;
    } data;

    /*--- References / Commands ---*/
    struct {
        float speed_ref;
        float speed_ref_target;
        float Iq_ref;
        float Iq_ref_target;
        float Id_ref;
        float Id_ref_target;
        float Vq_ref;
        float Vq_ref_target;
    } cmd;

    /*--- Startup State ---*/
    struct {
        float theta;
        float omega;
        uint32_t counter;
    } startup;

    /*--- Configuration (Directly from Flash) ---*/
    FlashConfig_t cfg;

    /*--- ADC Calibration & Offsets ---*/
    struct {
        int32_t offset_a;
        int32_t offset_b;
        int32_t offset_c;
        int32_t offset_vphase_a;
        int32_t offset_vphase_b;
        int32_t offset_vphase_c;
        uint16_t cal_samples;
    } adc_cal;

    /*--- Board Noise Profile & Signal Integrity ---*/
    struct {
        float noise_rms;        /**< Current measurement RMS noise floor [A] */
        float noise_pk_pk;      /**< Current measurement Peak-to-Peak noise [A] */
        float is_flat_thr;      /**< Auto-adapted settling threshold for Motor ID [A] */
        float i_inj_min;        /**< Minimum AC injection current for SNR > 20dB [A] */
        float bemf_noise_sq;    /**< BEMF magnitude noise floor threshold [V^2] */
        uint8_t health_status;  /**< Hardware health: 0=EXCELLENT, 1=GOOD, 2=NOISY, 3=FAULT */
    } noise_profile;

    /*--- Constraints & Performance ---*/
    float max_duty;
    uint32_t isr_time_cycles;

    /*--- Plotting (Telemetry Snapshot) ---*/
    struct {
        volatile uint8_t enabled;
        volatile uint8_t ready;
        float Vd, Vq;
        float Id, Iq;
        float Iq_ref, theta_elec;
        float Ia, Ib, Ic;
        float duty_a, duty_b, duty_c;
    } plot;

} FOC_Control_t;

/*===========================================================================*/
/* Global FOC Instance                                                       */
/*===========================================================================*/

extern FOC_Control_t g_foc;

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

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
 * @param speed_rpm Target speed in RPM
 */
void FOC_SetSpeedRef(float speed_rpm);

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
 * @brief Get current FOC state
 * @return Current state
 */
FOC_State_t FOC_GetState(void);

/**
 * @brief Get current fault code
 * @return Fault code (0 = no fault)
 */
FOC_Fault_t FOC_GetFault(void);

/**
 * @brief Clear fault and return to IDLE
 */
void FOC_ClearFault(void);

/**
 * @brief Enable/disable gate drivers
 * @param enable 1 to enable, 0 to disable
 */
void FOC_EnableDrivers(uint8_t enable);

/**
 * @brief Enable/disable gate driver
 * @param phase Phase to enable/disable
 * @param enable 1 to enable, 0 to disable
 */
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

void FOC_SlowTask(void);

/**
 * @brief Configure ADC Hardware Watchdogs based on FOC protection parameters.
 */
void FOC_ConfigureAWD(void);

#endif /* FOC_STATE_MACHINE_H */

/**
 * @brief Play a tune
 */
void playTune(void);