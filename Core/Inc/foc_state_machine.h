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
#include "smo_observer.h"

/*===========================================================================*/
/* FOC State Definitions                                                     */
/*===========================================================================*/

typedef enum {
    FOC_STATE_IDLE = 0,       /**< Motor stopped, waiting for start command */
    FOC_STATE_CALIBRATION,    /**< ADC offset calibration */
    FOC_STATE_ALIGN,          /**< Rotor alignment */
    FOC_STATE_STARTUP,        /**< Open-loop ramp-up */
    FOC_STATE_RUN,            /**< Closed-loop FOC running */
    FOC_STATE_STOP,           /**< Controlled stop */
    FOC_STATE_FAULT,          /**< Fault condition */
    FOC_STATE_SELF_COMMISSION /**< Motor parameter identification */
} FOC_State_t;

typedef enum {
    FOC_MODE_SPEED = 0, /**< Speed control mode */
    FOC_MODE_TORQUE     /**< Torque (current) control mode */
} FOC_ControlMode_t;

typedef enum {
    FOC_FAULT_NONE = 0,
    FOC_FAULT_OVERCURRENT,
    FOC_FAULT_OVERVOLTAGE,
    FOC_FAULT_UNDERVOLTAGE,
    FOC_FAULT_STARTUP_FAIL,
    FOC_FAULT_OBSERVER_FAIL,
    FOC_FAULT_STALL
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
        uint32_t run_counter;
    } status;

    /*--- Controllers ---*/
    struct {
        PI_Controller_t id;
        PI_Controller_t iq;
        PI_Controller_t speed;
        SMO_Observer_t smo;
        BIST_State_t bist;
    } ctrl;

    /*--- Live Data / Signals ---*/
    struct {
        float Ia, Ib, Ic;
        float Ialpha, Ibeta;
        float Id, Iq;
        float Vd, Vq;
        float Valpha, Vbeta;
        float theta_elec;
        float omega_elec;
        float speed_rpm;
        float Vbus;
        float duty_a, duty_b, duty_c;
    } data;

    /*--- References / Commands ---*/
    struct {
        float speed_ref;
        float speed_ref_target;
        float Iq_ref;
        float Iq_ref_target;
        float Id_ref;
        float Id_ref_target;
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
        uint16_t cal_samples;
    } adc_cal;

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
 * @param adc_ia Raw ADC value for phase A current
 * @param adc_ib Raw ADC value for phase B current
 * @param adc_ic Raw ADC value for phase C current
 * @param adc_vbus Raw ADC value for DC bus voltage
 */
void FOC_HighFrequencyTask(uint16_t adc_ia, uint16_t adc_ib, uint16_t adc_ic, uint16_t adc_vbus);

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

#endif /* FOC_STATE_MACHINE_H */
