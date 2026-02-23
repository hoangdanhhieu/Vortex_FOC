/**
 * @file bist_profiler.h
 * @brief Built-In Self-Test (BIST) Profiler for PID simulation
 */

#ifndef BIST_PROFILER_H
#define BIST_PROFILER_H

#include <stdint.h>

#include "foc_config.h"

typedef enum {
    BIST_MODE_OFF = 0,
    BIST_MODE_STEP = 1,
    BIST_MODE_CHIRP = 2,
    BIST_MODE_NOISE = 3
} BIST_Mode_t;

typedef struct {
    BIST_Mode_t mode;
    float amplitude;
    float offset;
    float frequency; /**< Step freq or max Chirp freq [Hz] */

    /* Internal State */
    float phase;
    float chirp_rate;
    float out_val;
    uint32_t prbs_state;   /**< LFSR state for noise generation */
    uint32_t tick_counter; /**< Downsampling counter (48kHz -> 8kHz) */
} BIST_State_t;

void BIST_Init(BIST_State_t* bist);
void BIST_Start(BIST_State_t* bist, uint8_t mode, float amp, float offset, float freq);
void BIST_Update(BIST_State_t* bist, float* target_ref);

#endif  // BIST_PROFILER_H
