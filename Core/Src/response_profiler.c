/**
 * @file response_profiler.c
 * @brief Dynamic Response Profiler & Test Signal Generator (Step, Chirp, PRBS Noise) for FOC
 */

#include "response_profiler.h"

#include <math.h>

#include "cordic_math.h"
#include "foc.h"
#include "foc_config.h"
#include "foc_state_machine.h"

/* We want to simulate an 8kHz update rate (e.g. DSHOT update rate).
 * At dynamic PWM frequency, downsample ratio is dynamically computed.
 */
#define PROFILER_UPDATE_RATE_HZ 8000.0f
#define PROFILER_DT (1.0f / PROFILER_UPDATE_RATE_HZ)

/* PRBS (Pseudo-Random Binary Sequence) generator for Noise Mode */
static float Profiler_GeneratePRBS(uint32_t* lfsr) {
    /* 32-bit LFSR (Galois) */
    uint32_t bit = ((*lfsr >> 0) ^ (*lfsr >> 2) ^ (*lfsr >> 6) ^ (*lfsr >> 7)) & 1U;
    *lfsr = (*lfsr >> 1) | (bit << 31);

    /* Map to [-1.0, 1.0] */
    return ((float)(*lfsr) / 2147483648.0f) - 1.0f;
}

void Profiler_Init(Profiler_State_t* profiler) {
    profiler->mode = PROFILER_MODE_OFF;
    profiler->amplitude = 0.0f;
    profiler->offset = 0.0f;
    profiler->frequency = 0.0f;

    profiler->phase = 0.0f;
    profiler->chirp_rate = 0.0f;
    profiler->out_val = 0.0f;
    profiler->prbs_state = 0xACE1ACE1U; /* Non-zero seed */
    profiler->tick_counter = 0;
}

void Profiler_Start(Profiler_State_t* profiler, uint8_t mode, float amp, float offset, float freq) {
    profiler->mode = (Profiler_Mode_t)mode;
    profiler->amplitude = amp;
    profiler->offset = offset;
    profiler->frequency = freq;

    profiler->phase = 0.0f;
    profiler->tick_counter = 0;

    /* Chirp logic: sweep from 1Hz to target 'freq' over 2 seconds */
    if (profiler->mode == PROFILER_MODE_CHIRP) {
        profiler->chirp_rate = (freq - 1.0f) / 2.0f;
    }
}

CCMRAM_FUNC void Profiler_Update(Profiler_State_t* profiler, float* target_ref) {
    if (profiler->mode == PROFILER_MODE_OFF) {
        return;
    }

    /* Downsample to 8kHz rate */
    uint32_t downsample_ratio = (uint32_t)(g_foc.cfg.pwm_frequency / PROFILER_UPDATE_RATE_HZ);
    if (downsample_ratio < 1) downsample_ratio = 1;
    profiler->tick_counter++;
    if (profiler->tick_counter < downsample_ratio) {
        /* Hold previous value in between 8kHz updates to simulate ZOH (Zero-Order Hold) of DSHOT */
        *target_ref = profiler->offset + profiler->out_val;
        return;
    }
    profiler->tick_counter = 0;

    switch (profiler->mode) {
        case PROFILER_MODE_STEP: {
            float phase_step = profiler->frequency * PROFILER_DT;
            profiler->phase += phase_step;
            if (profiler->phase >= 1.0f) {
                profiler->phase -= 1.0f;
            }
            profiler->out_val =
                (profiler->phase < 0.5f) ? profiler->amplitude : -profiler->amplitude;
            break;
        }

        case PROFILER_MODE_CHIRP: {
            /* Instantaneous frequency increases linearly */
            float current_freq = 1.0f + profiler->chirp_rate * profiler->phase *
                                            2.0f; /* phase scales time roughly */
            float phase_step = current_freq * PROFILER_DT;
            profiler->phase += phase_step;

            /* Reset chirp every 2 seconds roughly */
            if (profiler->phase >= 2.0f) {
                profiler->phase = 0.0f;
            }

            /* Normalize angle: (TWO_PI * phase * freq * 0.5) / PI = phase * freq */
            float angle_raw = profiler->phase * current_freq;
            float angle_norm = normalize_angle_norm(angle_raw);
            float dummy_cos, chirp_sin;
            cordic_sincos(angle_norm, &dummy_cos, &chirp_sin);
            profiler->out_val = profiler->amplitude * chirp_sin;
            break;
        }

        case PROFILER_MODE_NOISE: {
            profiler->out_val = profiler->amplitude * Profiler_GeneratePRBS(&profiler->prbs_state);
            break;
        }

        default: {
            profiler->out_val = 0.0f;
            break;
        }
    }

    *target_ref = profiler->offset + profiler->out_val;
}
