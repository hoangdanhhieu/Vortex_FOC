/**
 * @file motor_params.h
 * @brief Motor reference profile for RS2205 2300KV BLDC motor.
 *
 * NOTE: Vortex FOC identifies motor parameters online via Motor ID
 * or loads them from Flash. This file serves as an offline reference profile.
 */

#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

/*===========================================================================*/
/* Motor Physical Parameters (Reference Profile: RS2205 2300KV)              */
/*===========================================================================*/

/** Number of pole pairs (14 poles = 7 pole pairs) */
#define MOTOR_POLE_PAIRS 7

/** KV rating [RPM/V] */
#define MOTOR_KV 2300

/** Phase resistance [Ohm] */
#define MOTOR_RS 0.088252f

/** Phase inductance [H] */
#define MOTOR_LS 1.2e-5f

/** BEMF constant Ke [V/(rad/s)] = 60 / (sqrt(3) * 2 * PI * KV * PP) */
#define MOTOR_KE (60.0f / ((float)MOTOR_KV * 1.7320508f * 2.0f * 3.14159265f * (float)MOTOR_POLE_PAIRS))

/** Flux linkage [Wb] = Ke (for PMSM) */
#define MOTOR_FLUX_LINKAGE MOTOR_KE

/*===========================================================================*/
/* Motor Operating & Mechanical Limits                                       */
/*===========================================================================*/

/** Maximum phase current [A] */
#define MOTOR_MAX_CURRENT 30.0f

/** Saturation current [A] (current where Ls drops to 50%) */
#define MOTOR_ISAT 25.0f

/** Saturation coefficient alpha [1/A^2] = 1 / (Isat^2) */
#define MOTOR_ALPHA (1.0f / (MOTOR_ISAT * MOTOR_ISAT))

/** Maximum mechanical speed [RPM] */
#define MOTOR_MAX_SPEED_RPM 30000.0f

/** Rotor + Propeller moment of inertia [kg*m^2] */
#define MOTOR_INERTIA 1.5e-6f

#endif /* MOTOR_PARAMS_H */
