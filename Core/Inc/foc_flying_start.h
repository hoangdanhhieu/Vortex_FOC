/**
 * @file foc_flying_start.h
 * @brief FOC Flying Start, BEMF Tracking & Spinning PLL Lock
 */

#ifndef FOC_FLYING_START_H
#define FOC_FLYING_START_H

#include <stdint.h>

/**
 * @brief Initialize flying start detection parameters and counters
 */
void FOC_FlyingStart_Init(void);

/**
 * @brief State handler for FOC_STATE_DETECT (detects if motor is spinning via BEMF)
 */
void FOC_StateDetect(void);

/**
 * @brief State handler for FOC_STATE_FLYING_START (locks SMO PLL to spinning BEMF and transitions to RUN)
 */
void FOC_StateFlyingStart(void);

#endif /* FOC_FLYING_START_H */
