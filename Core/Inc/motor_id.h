/**
 * @file motor_id.h
 * @brief Motor Parameter Identification — Public API
 *
 * Rs: PI-controlled DC 2-point injection. Eliminates dead-time error via subtraction.
 * Ls: Dual-frequency AC sine injection with self-calibrating hardware delay compensation.
 *     The delay is solved from the constraint: Ls_compensated(f1) == Ls_compensated(f2).
 */

#ifndef MOTOR_ID_H
#define MOTOR_ID_H

#include <stdint.h>

/*===========================================================================*/
/* State Enum                                                                */
/*===========================================================================*/

typedef enum {
    MOTOR_ID_STATE_IDLE = 0,
    MOTOR_ID_STATE_ALIGN,               /* Safe 30 V/s voltage ramp alignment       */
    MOTOR_ID_STATE_MEASURE_RS,          /* 2-point DC Rs measurement (Settle-Ramp)  */
    MOTOR_ID_STATE_FREQ_DETECT,         /* 10 ms probe at 1000 Hz to select freq   */
    MOTOR_ID_STATE_MEASURE_SAT_PROFILE, /* 6-level DC bias + AC injection + ZOH Inv */
    MOTOR_ID_STATE_MEASURE_INERTIA,     /* Closed-loop speed step integral */
    MOTOR_ID_STATE_COMPLETE,
    MOTOR_ID_STATE_ERROR
} MotorID_State_t;

/*===========================================================================*/
/* Result / Debug Struct                                                     */
/*===========================================================================*/

typedef struct {
    float measured_rs;    /* Phase resistance               [Ω]     */
    float measured_ls;    /* Nominal phase inductance L0    [H]     */
    float sat_isat;       /* Saturation current Isat        [A]     */
    float sat_alpha;      /* Saturation coefficient alpha   [1/A^2] */
    float measured_vdead; /* Identified dead-time voltage   [V]     */
    float identified_deadtime_ns; /* Identified dead-time   [ns]    */
    float selected_freq_hz; /* Selected AC injection freq   [Hz]    */
    float measured_flux;    /* Identified permanent magnet flux [Wb]*/
    float measured_kv;      /* Calculated Motor KV              [RPM/V]*/
    float measured_b0;      /* Identified system gain b0        [rad/s^2 / A] */
    float measured_inertia; /* Identified mechanical inertia J  [kg.m^2] */
    
    MotorID_State_t state;
    uint32_t error_code;  /* 0 = OK, 1=Rs invalid, 2=Ls invalid, 3=Fit error, 7=No delta */
} MotorID_Result_t;

extern MotorID_Result_t id_result;


/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

void MotorID_Init(void);
void MotorID_Start(void);
void MotorID_Stop(void);

/**
 * @brief Initiate Offline Flux Measurement (Spin & Coast)
 */
void MotorID_MeasureFluxOffline(void);

/**
 * @brief Check if currently doing offline flux measurement
 */
uint8_t MotorID_IsFluxMeasuring(void);

/**
 * @brief State handler for Freewheeling Flux Measurement
 */
void FOC_StateCoastFluxID(void);

/**
 * @brief Initiate Offline Inertia Measurement (Speed Step Integral)
 */
void MotorID_MeasureInertiaOffline(void);

/**
 * @brief 1kHz Slow Task hook for Inertia Measurement
 */
void MotorID_InertiaSlowTask(void);

/**
 * @brief Run one step of the identification state machine.
 *        Called once per PWM ISR cycle (48 kHz).
 */
void MotorID_RunStep(float id, float iq, float vbus, float* vd, float* vq);

void MotorID_GetResults(MotorID_Result_t* results);

#endif /* MOTOR_ID_H */
