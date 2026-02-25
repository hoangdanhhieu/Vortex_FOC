/**
 * @file foc_config.h
 * @brief FOC system configuration and hardware defines
 */

#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* Place function in CCM SRAM for zero wait-state execution */
#define CCMRAM_FUNC __attribute__((section(".ccmram")))

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
#include "motor_params.h"

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
#define TIM1_ARR 1769

/** TIM1 counter max */
#define TIM1_COUNTER_MAX TIM1_ARR

/** Dead-time duration in nanoseconds (for compensation) */
#define DEAD_TIME_NS 0.0f

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
#define ADC_SAMPLE_CYCLES 6.5f
#define ADC_CONV_CYCLES 12.5f
#define ADC_CYCLES_PER_CH (ADC_SAMPLE_CYCLES + ADC_CONV_CYCLES) /* 19 */

/** Injected ranks per ADC (dual simultaneous → ranks run sequentially) */
#define ADC_INJ_RANKS 2U

/** Oversampling ratio */
#define ADC_OVS_RATIO 1U

/** Total ADC conversion time [s] */
#define ADC_TOTAL_TIME_S \
    (ADC_CYCLES_PER_CH * (float)ADC_INJ_RANKS * (float)ADC_OVS_RATIO / ADC_CLK_HZ)

/** ADC time expressed in TIM1 ticks (timer clocked at SYSCLK) */
#define ADC_TICKS ((uint32_t)(ADC_TOTAL_TIME_S * (float)SYSCLK_FREQ + 0.5f))

/** Safety margin [ticks] for ringing / settling / propagation */
#define ADC_MARGIN_DEFAULT 30.0f

/** Default TIM1 CH4 compare value for ADC trigger at boot */
#define TIM1_CH4_TRIGGER_DEFAULT (TIM1_ARR - ADC_TICKS - (uint32_t)ADC_MARGIN_DEFAULT)

/*===========================================================================*/
/* Current Sensing Configuration                                             */
/*===========================================================================*/

/** Shunt resistance [Ohm] */
#define SHUNT_RESISTANCE 0.001f

/** OPAMP gain */
#define OPAMP_GAIN 32.0f

/** ADC resolution (12-bit) */
#define ADC_RESOLUTION 4096

/** Current offset (bias point) at 0A [ADC counts] ~= Vref/2 */
#define ADC_CURRENT_OFFSET 2048

/** Current conversion factor: I = (ADC - offset) * factor */
/** factor = Vref / (ADC_res * Gain * R_shunt) */
/* 1.022... is an empirical calibration factor derived from measurements */
#define ADC_TO_CURRENT \
    (-(ADC_Vref / ((float)ADC_RESOLUTION * OPAMP_GAIN * SHUNT_RESISTANCE)) * 1.022556391f)

/* IIR Lowpass filter coefficient (0 < ALPHA <= 1)
 * Smaller ALPHA = stronger filtering, more delay
 * Larger ALPHA = weaker filtering, faster response
 */
#define CURRENT_FILTER_ALPHA 1.0f

#define FOC_DQ_FILTER_CUTOFF_RAD (CURRENT_LOOP_BW * 8.0f)
#define FOC_DQ_FILTER_ALPHA (FOC_DQ_FILTER_CUTOFF_RAD * CONTROL_PERIOD)
#define FOC_DQ_FILTER_TAU (1.0f / FOC_DQ_FILTER_CUTOFF_RAD)

/*===========================================================================*/
/* Vbus Measurement Configuration                                            */
/*===========================================================================*/

/** Vbus voltage divider: R_high / R_low */
#define VBUS_R_HIGH 10000.0f
#define VBUS_R_LOW 2200.0f

/** Vbus divider ratio */
#define VBUS_DIVIDER_RATIO ((VBUS_R_HIGH + VBUS_R_LOW) / VBUS_R_LOW)

/** Vbus conversion factor: Vbus = ADC * factor */
#define ADC_TO_VBUS ((ADC_Vref / (float)ADC_RESOLUTION) * VBUS_DIVIDER_RATIO)

/*===========================================================================*/
/* Speed Ramp Configuration                                                  */
/*===========================================================================*/

/** Maximum acceleration rate [RPM/s] */
#define SPEED_RAMP_ACCEL 20000.0f

/** Maximum deceleration rate [RPM/s] (positive value) */
#define SPEED_RAMP_DECEL 20000.0f

/** Current reference ramp rate [A/s] */
#define CURRENT_RAMP_RATE 500.0f

/*===========================================================================*/
/* PI Controller Default Gains                                               */
/*===========================================================================*/

/** Current loop bandwidth [Hz]*/
#define CURRENT_LOOP_BW 4800.0f

/** Speed loop bandwidth [rad/s]*/
#define SPEED_LOOP_BW 300.0f

/** Current PI controller gains (Kp = Ls * BW, Ki = Rs * BW) */
#define PI_ID_KP (MOTOR_LS * CURRENT_LOOP_BW)
#define PI_ID_KI (MOTOR_RS * CURRENT_LOOP_BW)
#define PI_IQ_KP PI_ID_KP
#define PI_IQ_KI PI_ID_KI

/** Current PI output limits [V] */
#define PI_CURRENT_OUT_MAX (MOTOR_VBUS_NOMINAL * 0.577f) /* Vbus/sqrt(3) */
#define PI_CURRENT_OUT_MIN (-PI_CURRENT_OUT_MAX)

/** Current PI integral limits [V] - same as output for current loops */
#define PI_CURRENT_INT_MAX PI_CURRENT_OUT_MAX
#define PI_CURRENT_INT_MIN PI_CURRENT_OUT_MIN

/** Speed PI controller gains */
#define PI_SPEED_KP 0.0008f
#define PI_SPEED_KI 0.0003f

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

/** SMO low-pass filter cutoff Hz */
#define SMO_BEMF_CUTOFF 8000.0f
/** SMO PLL bandwidth Hz */
#define SMO_PPL_CUTOFF 500.0f

/** SMO PLL integral limits [rad/s] - based on max expected electrical speed */
/** Max mech RPM * pole_pairs * 2*PI/60 = max electrical rad/s */
#define SMO_PLL_INT_MAX (MOTOR_MAX_SPEED_RPM * (TWO_PI / 60.0f) * (float)MOTOR_POLE_PAIRS)
#define SMO_PLL_INT_MIN (-SMO_PLL_INT_MAX)

#define COMP_DELAY_SAMPLES 2.0f
/*===========================================================================*/
/* Startup Configuration                                                     */
/*===========================================================================*/

/** Alignment current [A] */
#define ALIGN_CURRENT 0.6f

/** Alignment duration [ms] */
#define ALIGN_DURATION_MS 300

/** Open-loop startup current [A] */
#define STARTUP_CURRENT 0.2f

/** Open-loop startup voltage [V] */
#define STARTUP_VOLTAGE_MIN 0.5f
#define STARTUP_VOLTAGE_MAX 1.0f

/** Startup acceleration [RPM/s] */
#define STARTUP_ACCEL 500.0f

/** Minimum speed before switching to closed-loop [RPM] */
#define STARTUP_HANDOFF_SPEED 1000.0f

/** Transition blend duration from open-loop to closed-loop [ms] */
#define TRANSITION_BLEND_MS 50

/** Startup timeout [ms] - set to 0 to disable */
#define STARTUP_TIMEOUT_MS 0
/*===========================================================================*/
/* Safety / Fault Protection                                                 */
/*===========================================================================*/

/*--- Overcurrent protection ---*/
/** Software overcurrent threshold [A]**/
#define FAULT_OVERCURRENT_THRESHOLD 25.0f

/** Overcurrent deglitch: require N consecutive samples above threshold
 *  to avoid false trips from ADC noise. 1 = instant trip. */
#define FAULT_OVERCURRENT_COUNT 10

/*--- Bus voltage protection ---*/
/** Overvoltage threshold [V]*/
#define FAULT_OVERVOLTAGE_THRESHOLD 18.0f

/** Undervoltage threshold [V]*/
#define FAULT_UNDERVOLTAGE_THRESHOLD 8.0f

/*--- Stall / locked-rotor protection ---*/
/** Enable stall detection (0 = disable) */
#define FAULT_STALL_ENABLE 1

/** Stall is detected when |speed| < SPEED_THRESHOLD AND |Iq| > CURRENT_THRESHOLD
 *  persists for longer than TIME_MS. */
#define FAULT_STALL_SPEED_RPM 100.0f
#define FAULT_STALL_CURRENT_A 0.5f
#define FAULT_STALL_TIME_MS 500

/*===========================================================================*/
/* Motor Identification Configuration                                        */
/*===========================================================================*/

#define MOTOR_ID_ALIGN_I_DEFAULT 10.0f
#define MOTOR_ID_ALIGN_MS_DEFAULT 100.0f

#define MOTOR_ID_RS_SAMP_DEFAULT 100.0f
#define MOTOR_ID_RS_DELAY_DEFAULT 500.0f
#define MOTOR_ID_RS_I1_DEFAULT 3.0f
#define MOTOR_ID_RS_I2_DEFAULT 8.0f

#define MOTOR_ID_LS_DELAY_DEFAULT 100.0f
#define MOTOR_ID_LS_V_DEFAULT 2.0f
#define MOTOR_ID_LS_SAMP_DEFAULT 10.0f
#define MOTOR_ID_LS_DECAY_DEFAULT 100.0f
#define MOTOR_ID_LS_PULSES_DEFAULT 10.0f

#define MOTOR_ID_RAMP_STEP_DEFAULT 0.01f
#define MOTOR_ID_VAL_CNT_DEFAULT 100.0f
#define MOTOR_ID_I_TOL_DEFAULT 0.5f

/*===========================================================================*/
/* Debug/Safety Configuration                                                */
/*===========================================================================*/

/** Enable automatic handoff from open-loop to closed-loop (0=stay in open-loop)
 */
#define ENABLE_CLOSED_LOOP_HANDOFF 1

/** Maximum runtime before auto-stop [ms] - set to 0 to disable */
#define DEBUG_RUN_TIMEOUT_MS 0

#endif /* FOC_CONFIG_H */
