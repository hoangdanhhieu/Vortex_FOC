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
    MOTOR_ID_STATE_ALIGN,
    MOTOR_ID_STATE_MEASURE_RS,   /* PI-DC 2-point Rs measurement             */
    MOTOR_ID_STATE_FREQ_DETECT,  /* Quick probe to select f1 and f2          */
    MOTOR_ID_STATE_MEASURE_LS_F1,/* AC lock-in at f1, stores raw phasors     */
    MOTOR_ID_STATE_MEASURE_LS_F2,/* AC lock-in at f2, stores raw phasors     */
    MOTOR_ID_STATE_EXTRACT,      /* Solve for delay N*, compute final Ls     */
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

    /* Dead-time estimate (from AC measurement) */
    float identified_v_err;       /* Identified dead-time voltage error [V] */
    float identified_deadtime_ns; /* Identified dead-time [ns]              */

    /* Self-calibrated hardware delay */
    float identified_delay_samples; /* Optimal N* that makes Ls(f1)==Ls(f2) */

    /* Frequency selection debug */
    float dbg_f1_hz;          /* Chosen f1 [Hz]                         */
    float dbg_f2_hz;          /* Chosen f2 [Hz]                         */
    float dbg_phi_detect_deg; /* Impedance angle measured during FREQ_DETECT [deg] */

    /* Raw (uncompensated) phasors — stored for EXTRACT */
    float dbg_rapp_raw_f1; /* Apparent R at f1, NO delay compensation  [Ω] */
    float dbg_ls_raw_f1;   /* Apparent Ls at f1, NO delay compensation [H] */
    float dbg_iamp_f1;     /* Current amplitude at f1                  [A] */
    float dbg_rapp_raw_f2; /* Apparent R at f2, NO delay compensation  [Ω] */
    float dbg_ls_raw_f2;   /* Apparent Ls at f2, NO delay compensation [H] */
    float dbg_iamp_f2;     /* Current amplitude at f2                  [A] */

    /* Compensated values at optimal N* (convergence check) */
    float dbg_ls_comp_f1; /* Ls at f1 after delay compensation with N* [H] */
    float dbg_ls_comp_f2; /* Ls at f2 after delay compensation with N* [H] */
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
