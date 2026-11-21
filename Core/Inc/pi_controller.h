/**
 * @file pi_controller.h
 * @brief PI controller with anti-windup for FOC current/speed loops
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <math.h>

#include "foc_config.h"
/**
 * @brief PI Controller structure
 */
typedef struct {
    float Kp;       /**< Proportional gain */
    float Ki;       /**< Integral gain */
    float ki_dt;    /**< Precomputed Ki * dt */
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

    float new_integral = pi->integral + pi->ki_dt * error;
    new_integral = clampf(new_integral, pi->int_min, pi->int_max);

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

static inline void PI_SetGains(PI_Controller_t* pi, float Kp, float Ki) {
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->ki_dt = Ki * pi->dt;
}

static inline void PI_SetTiming(PI_Controller_t* pi, float dt) {
    pi->dt = dt;
    pi->ki_dt = pi->Ki * dt;
}

static inline void PI_SetLimits(PI_Controller_t* pi, float out_min, float out_max) {
    pi->out_min = out_min;
    pi->out_max = out_max;
    if (pi->integral > out_max) {
        pi->integral = out_max;
    } else if (pi->integral < out_min) {
        pi->integral = out_min;
    }
}

static inline void PI_SetIntLimits(PI_Controller_t* pi, float int_min, float int_max) {
    pi->int_min = int_min;
    pi->int_max = int_max;
    if (pi->integral > int_max) {
        pi->integral = int_max;
    } else if (pi->integral < int_min) {
        pi->integral = int_min;
    }
}

#endif /* PI_CONTROLLER_H */
