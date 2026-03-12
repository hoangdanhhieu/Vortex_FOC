/**
 * @file pi_controller.h
 * @brief PI controller with anti-windup for FOC current/speed loops
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include "foc_config.h"
/**
 * @brief PI Controller structure
 */
typedef struct {
    float Kp;       /**< Proportional gain */
    float Ki;       /**< Integral gain */
    float integral; /**< Integral accumulator */
    float out_min;  /**< Output minimum limit */
    float out_max;  /**< Output maximum limit */
    float int_min;  /**< Integral minimum limit (anti-windup) */
    float int_max;  /**< Integral maximum limit (anti-windup) */
    float dt;       /**< Sample time [s] */
} PI_Controller_t;

/**
 * @brief Initialize PI controller
 * @param pi Pointer to PI controller structure
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param out_min Output minimum limit
 * @param out_max Output maximum limit
 * @param dt Sample time in seconds
 */
void PI_Init(PI_Controller_t* pi, float Kp, float Ki, float out_min, float out_max, float dt);

/**
 * @brief Reset PI controller (clear integral)
 * @param pi Pointer to PI controller structure
 */
void PI_Reset(PI_Controller_t* pi);

/**
 * @brief Update PI controller with new error
 * @param pi Pointer to PI controller structure
 * @param error Error signal (reference - feedback)
 * @return Controller output
 */
CCMRAM_FUNC static inline float PI_Update(PI_Controller_t* pi, float error) {
    float p_term = pi->Kp * error;

    float new_integral = pi->integral + pi->Ki * error * pi->dt;

    if (new_integral > pi->int_max) {
        new_integral = pi->int_max;
    } else if (new_integral < pi->int_min) {
        new_integral = pi->int_min;
    }

    float output = p_term + new_integral;

    if (output > pi->out_max) {
        output = pi->out_max;
        if (error < 0.0f) {
            pi->integral = new_integral;
        }
    } else if (output < pi->out_min) {
        output = pi->out_min;
        if (error > 0.0f) {
            pi->integral = new_integral;
        }
    } else {
        pi->integral = new_integral;
    }

    return output;
}

/**
 * @brief Set PI gains at runtime
 * @param pi Pointer to PI controller structure
 * @param Kp New proportional gain
 * @param Ki New integral gain
 */
void PI_SetGains(PI_Controller_t* pi, float Kp, float Ki);

/**
 * @brief Set PI output limits at runtime
 * @param pi Pointer to PI controller structure
 * @param out_min New minimum limit
 * @param out_max New maximum limit
 */
void PI_SetLimits(PI_Controller_t* pi, float out_min, float out_max);

/**
 * @brief Set PI integral limits at runtime (for anti-windup tuning)
 * @param pi Pointer to PI controller structure
 * @param int_min New integral minimum limit
 * @param int_max New integral maximum limit
 */
void PI_SetIntLimits(PI_Controller_t* pi, float int_min, float int_max);

#endif /* PI_CONTROLLER_H */
