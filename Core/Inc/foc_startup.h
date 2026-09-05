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
 * @brief Force transition to complete immediately (used by Flying Start)
 */
void FOC_Startup_ForceComplete(void);

/**
 * @brief State handler for FOC_STATE_ALIGN (aligns rotor to 0 electrical angle)
 */
void FOC_StateAlign(void);

/**
 * @brief State handler for FOC_STATE_STARTUP (accelerates rotor in open-loop I-f mode)
 */
void FOC_StateStartup(void);

/**
 * @brief Check if transition blending (open-loop to closed-loop) is currently active
 */
uint8_t FOC_IsInTransition(void);

/**
 * @brief Update transition blending angle and speed during FOC_STATE_RUN
 * @param smo_theta Estimated electrical angle from SMO
 * @param smo_omega Estimated electrical speed from SMO
 * @param smo_speed_rpm Estimated mechanical RPM from SMO
 */
CCMRAM_FUNC void FOC_Transition_Update(float smo_theta_park, float smo_theta_pwm, float smo_omega,
                                       float smo_speed_rpm);

/**
 * @brief Calculate recommended sensorless handoff speed based on physical noise floor
 * @return Recommended handoff speed in RPM
 */
float FOC_CalculateRecommendedHandoffRpm(void);
#endif /* FOC_STARTUP_H */
