/**
 * @file ladrc_controller.c
 * @brief Linear Active Disturbance Rejection Control (LADRC) Speed Controller Implementation
 */

#include "ladrc_controller.h"
#include <math.h>

void LADRC_Init(LADRC_Controller_t* ctrl, float omega_c, float omega_o, float b0,
                float out_min, float out_max, float dt) {
    ctrl->dt = dt;
    ctrl->out_min = out_min;
    ctrl->out_max = out_max;

    LADRC_SetGains(ctrl, omega_c, omega_o, b0);
    LADRC_Reset(ctrl);
}

void LADRC_Reset(LADRC_Controller_t* ctrl) {
    ctrl->z1 = 0.0f;
    ctrl->z2 = 0.0f;
    ctrl->u_prev = 0.0f;
}

void LADRC_SetGains(LADRC_Controller_t* ctrl, float omega_c, float omega_o, float b0) {
    ctrl->omega_c = omega_c;
    ctrl->omega_o = omega_o;
    ctrl->b0 = (fabsf(b0) > 1e-4f) ? b0 : 1000.0f;
    ctrl->b0_inv = 1.0f / ctrl->b0;

    ctrl->beta1 = 2.0f * omega_o;
    ctrl->beta2 = omega_o * omega_o;

    float denom = 1.0f + ctrl->beta1 * ctrl->dt + ctrl->beta2 * ctrl->dt * ctrl->dt;
    ctrl->D_inv = (fabsf(denom) > 1e-6f) ? (1.0f / denom) : 1.0f;
}

void LADRC_SetLimits(LADRC_Controller_t* ctrl, float out_min, float out_max) {
    ctrl->out_min = out_min;
    ctrl->out_max = out_max;
}

void LADRC_SeedState(LADRC_Controller_t* ctrl, float omega_init, float u_init) {
    ctrl->z1 = omega_init;
    ctrl->u_prev = u_init;
    /* In steady-state, derivative is zero: b0 * u + f = 0 => z2 = -b0 * u_init */
    ctrl->z2 = -ctrl->b0 * u_init;
}
