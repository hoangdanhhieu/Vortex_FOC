/**
 * @file foc_startup.h
 * @brief FOC Rotor Alignment, Open-Loop I-f Startup & Transition Blending
 */

#ifndef FOC_STARTUP_H
#define FOC_STARTUP_H

#include <stdint.h>

#include "foc_config.h"

/**
 * @brief Reset startup and transition state variables
 */
void FOC_Startup_Reset(void);

/**
 * @brief Force transition to complete immediately
 */
void FOC_Startup_ForceComplete(void);

/**
 * @brief Start smooth transition blending (used by Flying Start and standard startup handoff)
 * @param blend_duration_ms Duration of blending transition in milliseconds
 * @param handoff_id Initial d-axis current to blend down to zero
 */
void FOC_Startup_StartTransition(float blend_duration_ms, float handoff_id);

/**
 * @brief State handler for FOC_STATE_ALIGN (aligns rotor to 0 electrical angle)
 */
void FOC_StateAlign(void);

/**
 * @brief State handler for FOC_STATE_STARTUP (accelerates rotor in open-loop I-f mode)
 */
void FOC_StateStartup(void);

/**
 * @brief Update transition blending angle and speed during FOC_STATE_RUN
 * @param smo_theta_park Estimated electrical angle for Park from SMO
 * @param smo_theta_pwm  Estimated electrical angle for PWM from SMO
 * @param smo_omega      Estimated electrical speed from SMO
 */
CCMRAM_FUNC void FOC_Transition_Update(float smo_theta_park, float smo_theta_pwm, float smo_omega);

#endif /* FOC_STARTUP_H */
