/**
 * @file response_profiler.h
 * @brief Dynamic Response Profiler & Test Signal Generator (Step, Chirp, PRBS Noise) for FOC
 */

#ifndef RESPONSE_PROFILER_H
#define RESPONSE_PROFILER_H

#include <stdint.h>

typedef enum {
    PROFILER_MODE_OFF = 0,
    PROFILER_MODE_STEP = 1,
    PROFILER_MODE_CHIRP = 2,
    PROFILER_MODE_NOISE = 3
} Profiler_Mode_t;

/* Backward-compatibility alias */
typedef Profiler_Mode_t BIST_Mode_t;
#define BIST_MODE_OFF PROFILER_MODE_OFF
#define BIST_MODE_STEP PROFILER_MODE_STEP
#define BIST_MODE_CHIRP PROFILER_MODE_CHIRP
#define BIST_MODE_NOISE PROFILER_MODE_NOISE

typedef struct {
    Profiler_Mode_t mode;
    float amplitude;
    float offset;
    float frequency; /**< Step freq or max Chirp freq [Hz] */

    /* Internal State */
    float phase;
    float chirp_rate;
    float out_val;
    uint32_t prbs_state;   /**< LFSR state for noise generation */
    uint32_t tick_counter; /**< Downsampling counter (8kHz) */
} Profiler_State_t;

/* Backward-compatibility alias */
typedef Profiler_State_t BIST_State_t;
typedef Profiler_State_t ResponseProfiler_State_t;

void Profiler_Init(Profiler_State_t* profiler);
void Profiler_Start(Profiler_State_t* profiler, uint8_t mode, float amp, float offset, float freq);
void Profiler_Update(Profiler_State_t* profiler, float* target_ref);

/* Backward-compatibility function aliases */
#define BIST_Init Profiler_Init
#define BIST_Start Profiler_Start
#define BIST_Update Profiler_Update

#endif  // RESPONSE_PROFILER_H
