/**
 * @file bist_profiler.c
 * @brief Built-In Self-Test (BIST) Profiler for PID simulation
 */

#include "bist_profiler.h"

#include <math.h>

/* We want to simulate an 8kHz update rate.
 * If our control loop is 48kHz, we downsample by 6.
 */
#define BIST_UPDATE_RATE_HZ 8000.0f
#define BIST_DOWNSAMPLE_RATIO ((uint32_t)(CONTROL_FREQUENCY / BIST_UPDATE_RATE_HZ))
#define BIST_DT (1.0f / BIST_UPDATE_RATE_HZ)

/* PRBS (Pseudo-Random Binary Sequence) generator for Noise Mode */
static float BIST_GeneratePRBS(uint32_t* lfsr) {
    /* 32-bit LFSR (Galois) */
    uint32_t bit = ((*lfsr >> 0) ^ (*lfsr >> 2) ^ (*lfsr >> 6) ^ (*lfsr >> 7)) & 1U;
    *lfsr = (*lfsr >> 1) | (bit << 31);

    /* Map to [-1.0, 1.0] */
    return ((float)(*lfsr) / 2147483648.0f) - 1.0f;
}

void BIST_Init(BIST_State_t* bist) {
    bist->mode = BIST_MODE_OFF;
    bist->amplitude = 0.0f;
    bist->offset = 0.0f;
    bist->frequency = 0.0f;

    bist->phase = 0.0f;
    bist->chirp_rate = 0.0f;
    bist->out_val = 0.0f;
    bist->prbs_state = 0xACE1ACE1U; /* Non-zero seed */
    bist->tick_counter = 0;
}

void BIST_Start(BIST_State_t* bist, uint8_t mode, float amp, float offset, float freq) {
    bist->mode = (BIST_Mode_t)mode;
    bist->amplitude = amp;
    bist->offset = offset;
    bist->frequency = freq;

    bist->phase = 0.0f;
    bist->tick_counter = 0;

    /* Chirp logic: sweep from 1Hz to target 'freq' over 2 seconds */
    if (bist->mode == BIST_MODE_CHIRP) {
        bist->chirp_rate = (freq - 1.0f) / 2.0f;
    }
}

CCMRAM_FUNC void BIST_Update(BIST_State_t* bist, float* target_ref) {
    if (bist->mode == BIST_MODE_OFF) {
        return;
    }

    /* Downsample to 8kHz rate */
    bist->tick_counter++;
    if (bist->tick_counter < BIST_DOWNSAMPLE_RATIO) {
        /* Hold previous value in between 8kHz updates to simulate ZOH (Zero-Order Hold) of DSHOT */
        *target_ref = bist->offset + bist->out_val;
        return;
    }
    bist->tick_counter = 0;

    /* Update Phase */

    switch (bist->mode) {
        case BIST_MODE_STEP: {
            float phase_step = bist->frequency * BIST_DT;
            bist->phase += phase_step;
            if (bist->phase >= 1.0f) {
                bist->phase -= 1.0f;
            }
            /* 50% duty cycle square wave */
            bist->out_val = (bist->phase < 0.5f) ? bist->amplitude : -bist->amplitude;
            break;
        }

        case BIST_MODE_CHIRP: {
            /* Instantaneous frequency increases linearly */
            float current_freq =
                1.0f + bist->chirp_rate * bist->phase * 2.0f; /* phase scales time roughly */
            float phase_step = current_freq * BIST_DT;
            bist->phase += phase_step;

            /* Reset chirp every 2 seconds roughly */
            if (bist->phase >= 2.0f) {
                bist->phase = 0.0f;
            }

            bist->out_val = bist->amplitude * sinf(TWO_PI * bist->phase * current_freq * 0.5f);
            break;
        }

        case BIST_MODE_NOISE: {
            /* Generate PRBS noise scaled by amplitude */
            bist->out_val = bist->amplitude * BIST_GeneratePRBS(&bist->prbs_state);
            break;
        }

        default: {
            bist->out_val = 0.0f;
            break;
        }
    }

    /* Apply offset (e.g., base throttle/torque) */
    *target_ref = bist->offset + bist->out_val;
}
