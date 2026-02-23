/**
 * @file pi_controller.c
 * @brief PI controller implementation with anti-windup
 */

#include "pi_controller.h"

#include "foc_config.h"

void PI_Init(PI_Controller_t* pi, float Kp, float Ki, float out_min, float out_max, float dt) {
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->int_min = out_min;
    pi->int_max = out_max;
    pi->dt = dt;
}

void PI_Reset(PI_Controller_t* pi) {
    pi->integral = 0.0f;
}

CCMRAM_FUNC float PI_Update(PI_Controller_t* pi, float error) {
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

void PI_SetGains(PI_Controller_t* pi, float Kp, float Ki) {
    pi->Kp = Kp;
    pi->Ki = Ki;
}

void PI_SetLimits(PI_Controller_t* pi, float out_min, float out_max) {
    pi->out_min = out_min;
    pi->out_max = out_max;
    if (pi->integral > out_max) {
        pi->integral = out_max;
    } else if (pi->integral < out_min) {
        pi->integral = out_min;
    }
}

void PI_SetIntLimits(PI_Controller_t* pi, float int_min, float int_max) {
    pi->int_min = int_min;
    pi->int_max = int_max;
    if (pi->integral > int_max) {
        pi->integral = int_max;
    } else if (pi->integral < int_min) {
        pi->integral = int_min;
    }
}
