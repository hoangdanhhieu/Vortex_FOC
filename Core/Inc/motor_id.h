/**
 * @file motor_id.h
 * @brief Motor Parameter Identification (Self-Commissioning) Module
 */

#ifndef MOTOR_ID_H
#define MOTOR_ID_H

#include <stdint.h>

#include "foc_config.h"
/*===========================================================================*/
/* Data Structures                                                           */
/*===========================================================================*/

typedef enum {
    MOTOR_ID_STATE_IDLE = 0,
    MOTOR_ID_STATE_ALIGN,      /* Align rotor for measurement */
    MOTOR_ID_STATE_MEASURE_RS, /* Measure Resistance */
    MOTOR_ID_STATE_MEASURE_LS, /* Measure Inductance */
    MOTOR_ID_STATE_COMPLETE,   /* Measurement complete */
    MOTOR_ID_STATE_ERROR
} MotorID_State_t;

#define MOTOR_ID_MAX_LUT_SIZE 20
typedef struct {
    float measured_rs;   /* Measured Phase Resistance [Ohm] */
    float measured_ls;   /* Measured Phase Inductance [H] */
    float measured_flux; /* Measured Flux Linkage [Wb] (Not impl yet) */

    /* Ls Saturation LUT */
    float ls_lut_currents[MOTOR_ID_MAX_LUT_SIZE];
    float ls_lut_values[MOTOR_ID_MAX_LUT_SIZE];
    uint8_t ls_lut_count;

    MotorID_State_t state; /* Internal state of ID process */
    uint32_t error_code;   /* 0 = No error */
} MotorID_Result_t;

extern MotorID_Result_t id_result;
/*===========================================================================*/
/* Configuration                                                             */
/*===========================================================================*/
#define RAMP_DT CONTROL_PERIOD
#define CURRENT_FILTER_COEFF 0.1f
/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

/**
 * @brief Initialize the ID module
 */
void MotorID_Init(void);

/**
 * @brief Start the parameter identification process
 */
void MotorID_Start(void);

/**
 * @brief Stop/Cancel identification
 */
void MotorID_Stop(void);

/**
 * @brief Run one step of the ID state machine (Called at PWM rate)
 * @param id Measured D-axis current
 * @param iq Measured Q-axis current
 * @param vbus Bus voltage
 * @param out_vd Pointer to output Vd
 * @param out_vq Pointer to output Vq
 */
void MotorID_RunStep(float ia, float ib, float vbus, float* duty_a, float* duty_b, float* duty_c);

/**
 * @brief Get the latest results
 */
void MotorID_GetResults(MotorID_Result_t* results);

#endif /* MOTOR_ID_H */
