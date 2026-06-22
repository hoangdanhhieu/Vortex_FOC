/**
 * @file motor_id.h
 * @brief Motor Parameter Identification — Public API
 *
 * Rs: Two-point DC injection (duty centred at midpoint 0.5).
 * Ls: AC sinusoidal injection with single-frequency lock-in demodulation.
 */

#ifndef MOTOR_ID_H
#define MOTOR_ID_H

#include <stdint.h>

/*===========================================================================*/
/* State Enum                                                                */
/*===========================================================================*/

typedef enum {
    MOTOR_ID_STATE_IDLE = 0,
    MOTOR_ID_STATE_ALIGN,
    MOTOR_ID_STATE_PROBE,
    MOTOR_ID_STATE_MEASURE_AC1,
    MOTOR_ID_STATE_MEASURE_AC2,
    MOTOR_ID_STATE_COMPLETE,
    MOTOR_ID_STATE_ERROR
} MotorID_State_t;

/*===========================================================================*/
/* Result / Debug Struct                                                     */
/*===========================================================================*/

typedef struct {
    float measured_rs; /* Phase resistance  [Ω]  */
    float measured_ls; /* Phase inductance  [H]  */
    MotorID_State_t state;
    uint32_t error_code; /* 0 = OK */

    /* Identified Deadtime and Voltage Error */
    float identified_v_err;       /* Identified deadtime voltage error [V] */
    float identified_deadtime_ns; /* Identified deadtime [ns] */

    /* Debug AC1 */
    float dbg_ac1_Iamp;
    float dbg_ac1_Rapp;
    float dbg_ac1_Ls;

    /* Debug AC2 */
    float dbg_ac2_Iamp;
    float dbg_ac2_Rapp;
    float dbg_ac2_Ls;
} MotorID_Result_t;

extern MotorID_Result_t id_result;

/*===========================================================================*/
/* Configuration                                                             */
/*===========================================================================*/
#define CURRENT_FILTER_COEFF 0.01f

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

void MotorID_Init(void);
void MotorID_Start(void);
void MotorID_Stop(void);

/**
 * @brief Run one step of the identification state machine.
 *        Called once per PWM ISR cycle (48 kHz).
 */
void MotorID_RunStep(float id, float iq, float vbus, float* vd, float* vq);

void MotorID_GetResults(MotorID_Result_t* results);

#endif /* MOTOR_ID_H */
