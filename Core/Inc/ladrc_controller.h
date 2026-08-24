/**
 * @file ladrc_controller.h
 * @brief Linear Active Disturbance Rejection Control (LADRC) Speed Controller
 *
 * Implemented using Implicit Backward Euler Discretization for unconditional numerical stability.
 */

#ifndef LADRC_CONTROLLER_H
#define LADRC_CONTROLLER_H

#include "foc_config.h"

/*===========================================================================*/
/* LADRC Controller Structure                                                */
/*===========================================================================*/

typedef struct {
    float omega_c;  /**< Controller bandwidth [rad/s] */
    float omega_o;  /**< Observer bandwidth [rad/s] */
    float b0;       /**< Control gain scaling factor = 1.5 * p^2 * psi / J */
    float b0_inv;   /**< 1.0 / b0 */
    float dt;       /**< Sample time [s] (e.g. 0.001 s for 1 kHz task) */
    float out_min;  /**< Saturated minimum output command [A] */
    float out_max;  /**< Saturated maximum output command [A] */

    /* Pre-calculated Backward Euler LESO observer gains */
    float beta1;    /**< 2.0 * omega_o */
    float beta2;    /**< omega_o * omega_o */
    float D_inv;    /**< 1.0 / (1.0 + beta1 * dt + beta2 * dt * dt) */

    /* Internal states */
    float z1;       /**< Estimated electrical speed [rad/s] */
    float z2;       /**< Estimated total disturbance [rad/s^2] */
    float u_prev;   /**< Previous saturated control command [A] (for feedback anti-windup) */
} LADRC_Controller_t;

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

/**
 * @brief Initialize LADRC speed controller
 * @param ctrl Pointer to LADRC controller structure
 * @param omega_c Controller bandwidth in rad/s
 * @param omega_o Observer bandwidth in rad/s
 * @param b0 System control gain scaling factor
 * @param out_min Saturated minimum output limit
 * @param out_max Saturated maximum output limit
 * @param dt Sample time in seconds
 */
void LADRC_Init(LADRC_Controller_t* ctrl, float omega_c, float omega_o, float b0,
                float out_min, float out_max, float dt);

/**
 * @brief Reset LADRC controller states to zero
 * @param ctrl Pointer to LADRC controller structure
 */
void LADRC_Reset(LADRC_Controller_t* ctrl);

/**
 * @brief Update LADRC tuning parameters
 * @param ctrl Pointer to LADRC controller structure
 * @param omega_c Controller bandwidth in rad/s
 * @param omega_o Observer bandwidth in rad/s
 * @param b0 System control gain scaling factor
 */
void LADRC_SetGains(LADRC_Controller_t* ctrl, float omega_c, float omega_o, float b0);

/**
 * @brief Update LADRC output saturation limits
 * @param ctrl Pointer to LADRC controller structure
 * @param out_min Saturated minimum output limit
 * @param out_max Saturated maximum output limit
 */
void LADRC_SetLimits(LADRC_Controller_t* ctrl, float out_min, float out_max);

/**
 * @brief Seed LADRC states for bumpless handoff from Open-Loop to Closed-Loop
 * @param ctrl Pointer to LADRC controller structure
 * @param omega_init Initial electrical speed [rad/s]
 * @param u_init Initial Iq current command [A]
 */
void LADRC_SeedState(LADRC_Controller_t* ctrl, float omega_init, float u_init);

/**
 * @brief Execute one step of the LADRC speed controller (typically at 1 kHz)
 * @param ctrl Pointer to LADRC controller structure
 * @param omega_ref Electrical speed target [rad/s]
 * @param omega_fb Electrical speed feedback from observer [rad/s]
 * @return Saturated Iq_ref current command [A]
 */
CCMRAM_FUNC static inline float LADRC_Update(LADRC_Controller_t* ctrl, float omega_ref, float omega_fb) {
    /* 1. Update Linear Extended State Observer (LESO) via Implicit Backward Euler */
    float num = ctrl->z1 +
                ctrl->dt * ctrl->z2 +
                ctrl->dt * ctrl->b0 * ctrl->u_prev +
                ctrl->dt * (ctrl->beta1 + ctrl->beta2 * ctrl->dt) * omega_fb;
    float z1_new = num * ctrl->D_inv;

    ctrl->z2 = ctrl->z2 - ctrl->beta2 * ctrl->dt * (z1_new - omega_fb);
    ctrl->z1 = z1_new;

    /* 2. Proportional Error Tracking Control Law */
    float u0 = ctrl->omega_c * (omega_ref - ctrl->z1);

    /* 3. Total Disturbance Rejection */
    float u_raw = (u0 - ctrl->z2) * ctrl->b0_inv;

    /* 4. Output Saturation & Anti-Windup Tracking */
    float u_sat = clampf(u_raw, ctrl->out_min, ctrl->out_max);
    ctrl->u_prev = u_sat;

    return u_sat;
}

#endif /* LADRC_CONTROLLER_H */
