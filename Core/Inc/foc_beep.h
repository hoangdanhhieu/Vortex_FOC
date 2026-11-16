/**
 * @file foc_beep.h
 * @brief Table-driven, multi-frequency ESC beep sequencer & synthesizer
 */

#ifndef FOC_BEEP_H
#define FOC_BEEP_H

#include <stdint.h>

#include "foc_config.h"

/**
 * @brief Built-in ESC beep melody types
 */
typedef enum {
    FOC_BEEP_POWER_ON = 0, /**< Power-on ascending ready chime */
    FOC_BEEP_SIGNAL_LOCK,  /**< Input signal locked chime */
    FOC_BEEP_ARMED,        /**< Armed confirmation double-chirp */
    FOC_BEEP_BEACON_1,     /**< DShot Beacon 1: Standard alert rising tone */
    FOC_BEEP_BEACON_2,     /**< DShot Beacon 2: High alert falling tone */
    FOC_BEEP_BEACON_3,     /**< DShot Beacon 3: Lost-model 5-note warble */
    FOC_BEEP_BEACON_4,     /**< DShot Beacon 4: Triple staccato chirp */
    FOC_BEEP_BEACON_5,     /**< DShot Beacon 5: Low-pitch siren */
    FOC_BEEP_SUCCESS,      /**< Self-commission / Motor ID success chime */
    FOC_BEEP_ERROR,        /**< Non-hardware alert / ID error tone */
    FOC_BEEP_COUNT
} FOC_BeepType_t;

#define FOC_BEEP_BEACON FOC_BEEP_BEACON_3

/**
 * @brief Per-tick step status returned by FOC_Beep_Step()
 */
typedef enum {
    FOC_BEEP_STEP_DONE = 0, /**< Sequence complete or timed out */
    FOC_BEEP_STEP_SILENCE,  /**< Gap/pause interval (apply 0 V / 50% duty) */
    FOC_BEEP_STEP_TONE      /**< Active tone interval (apply Vd at theta) */
} FOC_BeepStepStatus_t;

/**
 * @brief Initialize and start a beep sequence
 * @param type Sequence identifier from FOC_BeepType_t
 * @param dt Control loop sample period [s]
 * @param motor_rs Stator phase resistance [Ohm]
 * @param motor_ls Stator phase inductance [H]
 * @param vbus Measured DC bus voltage [V]
 * @param deadtime_duty Normalized deadtime duty (deadtime_ns * 1e-9 * pwm_freq)
 * @param volume_pct User beep volume percentage [0..100%] (0 = mute)
 * @return 1 if sequence started, 0 if disabled, muted, or invalid
 */
uint8_t FOC_Beep_Start(FOC_BeepType_t type, float dt, float motor_rs, float motor_ls,
                       float vbus, float deadtime_duty, uint8_t volume_pct);

/**
 * @brief Stop the active beep sequence and clear internal state
 */
void FOC_Beep_Stop(void);

/**
 * @brief Advance the beep sequencer by one high-frequency ISR tick
 * @param out_vd Output d-axis excitation voltage [V] (valid when returning FOC_BEEP_STEP_TONE)
 * @param out_theta Output normalized electrical angle [-1, 1) (valid when FOC_BEEP_STEP_TONE)
 * @return Current step status
 */
CCMRAM_FUNC FOC_BeepStepStatus_t FOC_Beep_Step(float* out_vd, float* out_theta);

#endif /* FOC_BEEP_H */

