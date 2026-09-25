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

CCMRAM_FUNC void PI_Reset(PI_Controller_t* pi) {
    pi->integral = 0.0f;
}

