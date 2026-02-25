/**
 * @file motor_params.h
 * @brief Motor parameters for RS2205 2300KV BLDC motor
 *
 * NOTE: These are estimated values. For best performance,
 * measure Rs, Ls, and Ke on your actual motor.
 */

#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

/*===========================================================================*/
/* Motor Electrical Parameters (RS2205 2300KV - Estimated)                   */
/*===========================================================================*/

/** Number of pole pairs (14 poles = 7 pole pairs) */
#define MOTOR_POLE_PAIRS 7

/** KV rating [RPM/V] */
#define MOTOR_KV 2300

/** Phase resistance [Ohm] - MEASURE THIS ON YOUR MOTOR */
#define MOTOR_RS 0.070f

/** Phase inductance [H] - MEASURE THIS ON YOUR MOTOR */
#define MOTOR_LS 2.487e-5f

/** BEMF constant [V/(rad/s)] = 60 / (Sqrt3 * 2PI * KV * PP) */
#define MOTOR_KE (60.0f / (MOTOR_KV * 1.732 * 2 * 3.14159265f * MOTOR_POLE_PAIRS))

/** Flux linkage [Wb] = Ke (for PMSM) */
#define MOTOR_FLUX_LINKAGE MOTOR_KE

/*===========================================================================*/
/* Motor Mechanical/Operating Limits                                         */
/*===========================================================================*/

/** Maximum phase current [A] */
#define MOTOR_MAX_CURRENT 30.0f

/** Continuous phase current [A] */
#define MOTOR_CONT_CURRENT 15.0f

/** Maximum speed [RPM] */
#define MOTOR_MAX_SPEED_RPM 30000.0f

/** Minimum speed for sensorless operation [RPM] */
#define MOTOR_MIN_SPEED_RPM 1000.0f

/** Operating voltage range [V] */
#define MOTOR_VBUS_MIN 10.0f
#define MOTOR_VBUS_MAX 16.8f
#define MOTOR_VBUS_NOMINAL 12.8f

/*===========================================================================*/
/* Derived Constants (calculated at compile time)                            */
/*===========================================================================*/

/** Maximum electrical speed [rad/s] */
#define MOTOR_MAX_SPEED_ELEC_RAD \
    ((MOTOR_MAX_SPEED_RPM / 60.0f) * 2.0f * 3.14159265f * MOTOR_POLE_PAIRS)

/** Electrical time constant [s] = Ls / Rs */
#define MOTOR_ELEC_TIME_CONST (MOTOR_LS / MOTOR_RS)

#endif /* MOTOR_PARAMS_H */
