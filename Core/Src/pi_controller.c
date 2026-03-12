/**
 * @file pi_controller.c
 * @brief PI controller implementation with anti-windup
 */

#include "pi_controller.h"

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
