/**
 * @file foc_config.h
 * @brief FOC system configuration and hardware defines
 */

#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* Place function in CCM SRAM for zero wait-state execution */
#define CCMRAM_FUNC __attribute__((section(".ccmram")))

/* Place function in SRAM1 (.RamFunc)*/
#define RAM_FUNC __attribute__((section(".RamFunc")))

/*===========================================================================*/
/* Math Constants                                                            */
/*===========================================================================*/

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define RAD_TO_RPM (60.0f / TWO_PI)
#define RPM_TO_RAD (TWO_PI / 60.0f)

#define SQRT3 1.7320508075688772f
#define SQRT3_INV 0.5773502691896257f
#define TWO_THIRDS 0.6666666666666666f
#define ONE_THIRD 0.3333333333333333f

/**
 * @brief Fast float clamp helper to range [min_val, max_val]
 */
CCMRAM_FUNC static inline float clampf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief Fast float symmetric saturation helper to range [-max_val, max_val]
 */
CCMRAM_FUNC static inline float saturatef(float val, float max_val) {
    if (val < -max_val) return -max_val;
    if (val > max_val) return max_val;
    return val;
}

#include "motor_params.h"

extern volatile float ADC_Vref;
/*===========================================================================*/
/* Feature Toggles                                                           */
/*===========================================================================*/

/** Enable field weakening (set to 1 to enable) */
#define ENABLE_FIELD_WEAKENING 0

#define LOG_MODE_NONE 0
#define LOG_MODE_UART 1
#define LOG_MODE_SWO 2
#define LOG_MODE_USB 3
#define LOG_OUTPUT_MODE LOG_MODE_USB

/*===========================================================================*/
/* System Clock and PWM Configuration                                        */
/*===========================================================================*/

/** System clock frequency [Hz] */
#define SYSCLK_FREQ 170000000UL

/** PWM switching frequency [Hz] */
#define PWM_FREQUENCY 48000UL

/** Control loop frequency [Hz] */
#define CONTROL_FREQUENCY PWM_FREQUENCY

/** Control loop period [s] */
#define CONTROL_PERIOD (1.0f / (float)CONTROL_FREQUENCY)
#define CONTROL_PERIOD_F CONTROL_PERIOD

/** TIM1 Auto-reload value */
#define TIM1_ARR 1770

/** TIM1 counter max */
#define TIM1_COUNTER_MAX TIM1_ARR

/** Dead-time duration in nanoseconds */
#define DEAD_TIME_NS 480.0f
#define DEADTIME_NS_TO_TICKS(ns)                                                                 \
    ((uint8_t)(((ns) <= 747.0f)    ? ((uint32_t)((ns) * 170.0f / 1000.0f + 0.5f))                \
               : ((ns) <= 1494.0f) ? (0x80 | ((uint32_t)((ns) * 170.0f / 2000.0f + 0.5f) - 64))  \
               : ((ns) <= 2964.0f) ? (0xC0 | ((uint32_t)((ns) * 170.0f / 8000.0f + 0.5f) - 32))  \
               : ((ns) <= 5929.0f) ? (0xE0 | ((uint32_t)((ns) * 170.0f / 16000.0f + 0.5f) - 32)) \
                                   : 0xFF))
#define TIM1_DEADTIME_TICKS DEADTIME_NS_TO_TICKS(DEAD_TIME_NS)

#define DEAD_TIME_DUTY (DEAD_TIME_NS * 1e-9f * (float)PWM_FREQUENCY)

/*===========================================================================*/
/* ADC Trigger Timing → MAX_DUTY derivation                                  */
/*===========================================================================*/
/**
 * ADC clock = SYSCLK / ADC_PRESCALER
 *   → 1 ADC cycle = ADC_PRESCALER timer ticks
 *   → no need to convert through seconds
 */
#define ADC_PRESCALER 4U
#define ADC_CLK_HZ ((float)SYSCLK_FREQ / (float)ADC_PRESCALER) /* 42.5 MHz */

/** ADC cycles per channel: sampling + 12.5 conversion cycles (12-bit) */
#define ADC_SAMPLE_CYCLES 2.5f
#define ADC_CONV_CYCLES 12.5f
#define ADC_CYCLES_PER_CH (ADC_SAMPLE_CYCLES + ADC_CONV_CYCLES)

/** Injected ranks per ADC (dual simultaneous → ranks run sequentially) */
#define ADC_INJ_RANKS 1U

/** Oversampling ratio */
#define ADC_OVS_RATIO 1U

/** Total ADC conversion time [s] */
#define ADC_TOTAL_TIME_S \
    (ADC_CYCLES_PER_CH * (float)ADC_INJ_RANKS * (float)ADC_OVS_RATIO / ADC_CLK_HZ)

/** ADC time expressed in TIM1 ticks (timer clocked at SYSCLK) */
#define ADC_TICKS ((uint32_t)(ADC_TOTAL_TIME_S * (float)SYSCLK_FREQ + 0.5f))

/** Safety margin [ticks] for ringing / settling / propagation */
#define ADC_MARGIN_DEFAULT 10.0f

#define MAX_DUTY_HIGH 0.95

/*===========================================================================*/
/* Current Sensing Configuration                                             */
/*===========================================================================*/

/** Cutoff frequency for the Current Self-Tuning Filter [Hz] */
#define CURRENT_STF_FC 500.0f

/** Shunt resistance [Ohm] */
#define SHUNT_RESISTANCE 0.005f

/** OPAMP gain */
#define OPAMP_GAIN 7.0f

/** ADC resolution (12-bit) */
#define ADC_RESOLUTION 4096

/** Current offset (bias point) at 0A [ADC counts] ~= Vref/2 */
#define ADC_CURRENT_OFFSET 0

/** Current conversion factor: I = (ADC - offset) * factor */
/** factor = Vref / (ADC_res * Gain * R_shunt) */
/* 1.022... is an empirical calibration factor derived from measurements */
#define ADC_TO_CURRENT (-(1 / ((float)ADC_RESOLUTION * OPAMP_GAIN * SHUNT_RESISTANCE)))

/*===========================================================================*/
/* Vbus Measurement Configuration                                            */
/*===========================================================================*/

/** Vbus voltage divider: R_high / R_low */
#define VBUS_R_HIGH 15000.0f
#define VBUS_R_LOW 1000.0f

/** Vbus divider ratio */
#define VBUS_DIVIDER_RATIO ((VBUS_R_HIGH + VBUS_R_LOW) / VBUS_R_LOW)

/** Vbus conversion factor: Vbus = ADC * factor */
#define ADC_TO_VBUS ((ADC_Vref / (float)ADC_RESOLUTION) * VBUS_DIVIDER_RATIO)

/** Phase voltage measurement configuration (3-resistor divider with bias) */
#define PHASE_R_UP 15000.0f  /* Pull-up resistor to 3.3V (Vref) */
#define PHASE_R_IN 10000.0f  /* Series input resistor from phase voltage */
#define PHASE_R_DOWN 1000.0f /* Pull-down resistor to GND */

/** Phase voltage conversion gain and offset factors */
#define PHASE_VOLTAGE_GAIN (1.0f + (PHASE_R_IN / PHASE_R_UP) + (PHASE_R_IN / PHASE_R_DOWN))
#define PHASE_VOLTAGE_OFFSET_FACTOR (PHASE_R_IN / PHASE_R_UP)

/*===========================================================================*/
/* Speed Ramp Configuration                                                  */
/*===========================================================================*/

/** Maximum acceleration rate [RPM/s] */
#define SPEED_RAMP_ACCEL 20000.0f

/** Maximum deceleration rate [RPM/s] (positive value) */
#define SPEED_RAMP_DECEL 20000.0f

/** Current reference ramp rate [A/s] */
#define CURRENT_RAMP_RATE 50.0f

/*===========================================================================*/
/* PI Controller Default Gains                                               */
/*===========================================================================*/

/** Current loop bandwidth [Hz]*/
#define CURRENT_LOOP_BW 4800.0f

/** Voltage ramp rate default [V/s] */
#define VOLTAGE_RAMP_RATE 50.0f

/** Current PI controller gains (Kp = Ls * BW, Ki = Rs * BW) */
#define PI_ID_KP (MOTOR_LS * CURRENT_LOOP_BW)
#define PI_ID_KI (MOTOR_RS * CURRENT_LOOP_BW)
#define PI_IQ_KP PI_ID_KP
#define PI_IQ_KI PI_ID_KI

/** Speed PI controller gains */
#define PI_SPEED_KP 0.0008f
#define PI_SPEED_KI 0.002f

/** Speed PI output limits [A] (Iq reference) */
#define PI_SPEED_OUT_MAX MOTOR_CONT_CURRENT
#define PI_SPEED_OUT_MIN (-PI_SPEED_OUT_MAX)

/** Speed PI integral limits [A] - smaller than output to reduce overshoot */
#define PI_SPEED_INT_MAX (PI_SPEED_OUT_MAX * 0.5f) /* 50% of output max */
#define PI_SPEED_INT_MIN (-PI_SPEED_INT_MAX)

/*===========================================================================*/
/* SMO Observer Configuration                                                */
/*===========================================================================*/

/** SMO sliding gain - limited by L: k_slide * dt/L < few amps per step */
#define SMO_K_SLIDE 20.0f

/** SMO sigmoid bandwidth (smaller = sharper, better angle SNR but more chatter) */
#define SMO_K_SIGMOID 30.0f

/** SMO PLL bandwidth Hz */
#define SMO_PPL_CUTOFF 500.0f

/** SMO PLL integral limits [rad/s] - based on max expected electrical speed */
/** Max mech RPM * pole_pairs * 2*PI/60 = max electrical rad/s */
#define SMO_PLL_INT_MAX (MOTOR_MAX_SPEED_RPM * (TWO_PI / 60.0f) * (float)MOTOR_POLE_PAIRS)
#define SMO_PLL_INT_MIN (-SMO_PLL_INT_MAX)

#define COMP_DELAY_SAMPLES 0.0f

/** Vbus IIR low-pass filter coefficient (alpha = 0.95, ~16 Hz cutoff at 1 kHz) */
#define VBUS_IIR_ALPHA 0.95f

/** ADC channel switching hysteresis (duty difference threshold to prevent jitter) */
#define SKIP_HYSTERESIS 0.03f

/*===========================================================================*/
/* Startup Configuration                                                     */
/*===========================================================================*/

/** Alignment current [A] */
#define ALIGN_CURRENT 0.3f

/** Alignment duration [ms] */
#define ALIGN_DURATION_MS 500

/** Open-loop startup current [A] */
#define STARTUP_CURRENT 0.5f

/** Open-loop startup voltage [V] */
#define STARTUP_VOLTAGE_MIN 0.5f
#define STARTUP_VOLTAGE_MAX 1.0f

/** Startup acceleration [RPM/s] */
#define STARTUP_ACCEL 500.0f

/** Minimum speed before switching to closed-loop [RPM] */
#define STARTUP_HANDOFF_SPEED 1000.0f

/** Transition blend duration from open-loop to closed-loop [ms] */
#define TRANSITION_BLEND_MS 20.0f

/** Startup timeout [ms] - set to 0 to disable */
#define STARTUP_TIMEOUT_MS 0
/*===========================================================================*/
/* Safety / Fault Protection                                                 */
/*===========================================================================*/

/*--- Overcurrent protection ---*/
/** Software overcurrent threshold [A]**/
#define FAULT_OVERCURRENT_THRESHOLD 10.0f

/** Overcurrent deglitch: require N consecutive samples above threshold
 *  to avoid false trips from ADC noise. 1 = instant trip. */
#define FAULT_OVERCURRENT_COUNT 10

/*--- Bus voltage protection ---*/
/** Overvoltage threshold [V]*/
#define FAULT_OVERVOLTAGE_THRESHOLD 24.0f

/** Undervoltage threshold [V]*/
#define FAULT_UNDERVOLTAGE_THRESHOLD 12.0f

/*--- Stall / locked-rotor protection ---*/
/** Enable stall detection (0 = disable) */
#define FAULT_STALL_ENABLE 1

/** Stall is detected when |speed| < SPEED_THRESHOLD AND |Iq| > CURRENT_THRESHOLD
 *  persists for longer than TIME_MS. */
#define FAULT_STALL_SPEED_RPM 700.0f
#define FAULT_STALL_CURRENT_A 35.0f
#define FAULT_STALL_TIME_MS 100

/*--- Stop ramp-down ---*/
/** Maximum time for controlled stop ramp-down before forced shutdown [ms] */
#define STOP_TIMEOUT_MS 3000

/** Current threshold as percentage of motor max current */
#define STOP_CURRENT_PERCENT 2.0f

/*===========================================================================*/
/* Debug/Safety Configuration                                                */
/*===========================================================================*/

/** Enable automatic handoff from open-loop to closed-loop (0=stay in open-loop)
 */
#define ENABLE_CLOSED_LOOP_HANDOFF 1

/** Maximum runtime before auto-stop [ms] - set to 0 to disable */
#define DEBUG_RUN_TIMEOUT_MS 0

#define BEEP_PERIOD_TICKS (2 * CONTROL_FREQUENCY)
#define BEEP_DURATION_TICKS (CONTROL_FREQUENCY / 16)
#define BEEP_STEP_FREQ 4500.0f * CONTROL_PERIOD
#endif /* FOC_CONFIG_H */
