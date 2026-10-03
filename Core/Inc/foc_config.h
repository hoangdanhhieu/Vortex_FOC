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
#define SQRT2 1.4142135623730951f
#define SQRT3_INV 0.5773502691896257f

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

extern volatile float ADC_Vref;

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

/** Dead-time duration in nanoseconds */
#define DEAD_TIME_NS 400.0f
#define DEADTIME_NS_TO_TICKS(ns)                                                                 \
    ((uint8_t)(((ns) <= 747.0f)    ? ((uint32_t)((ns) * 170.0f / 1000.0f + 0.5f))                \
               : ((ns) <= 1494.0f) ? (0x80 | ((uint32_t)((ns) * 170.0f / 2000.0f + 0.5f) - 64))  \
               : ((ns) <= 2964.0f) ? (0xC0 | ((uint32_t)((ns) * 170.0f / 8000.0f + 0.5f) - 32))  \
               : ((ns) <= 5929.0f) ? (0xE0 | ((uint32_t)((ns) * 170.0f / 16000.0f + 0.5f) - 32)) \
                                   : 0xFF))
#define TIM1_DEADTIME_TICKS DEADTIME_NS_TO_TICKS(DEAD_TIME_NS)

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
#define ADC_MARGIN_DEFAULT 1.0f

#define MAX_DUTY_HIGH 0.95

/*===========================================================================*/
/* Current Sensing Configuration                                             */
/*===========================================================================*/

/** Cutoff frequency for the Current Self-Tuning Filter [Hz] */
#define CURRENT_STF_FC 1500.0f

/** Shunt resistance [Ohm] */
#define SHUNT_RESISTANCE 0.005f

/** OPAMP gain */
#define OPAMP_GAIN 15.0f

/** ADC resolution (12-bit) */
#define ADC_RESOLUTION 4096

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

/** Maximum acceleration rate [rad/s^2 elec] */
#define SPEED_RAMP_ACCEL 15000.0f

/** Maximum deceleration rate [rad/s^2 elec] (positive value) */
#define SPEED_RAMP_DECEL 15000.0f

/** Current reference ramp rate [A/s] */
#define CURRENT_RAMP_RATE 50.0f

/*===========================================================================*/
/* PI Controller Default Gains                                               */
/*===========================================================================*/

/** Current loop bandwidth [Hz]*/
#define CURRENT_LOOP_BW 4800.0f * TWO_PI

/** Voltage ramp rate default [V/s] */
#define VOLTAGE_RAMP_RATE 50.0f

/** Voltage mode regenerative braking current limit [A] */
#define VOLTAGE_MODE_REGEN_CURRENT_MAX 1.5f

/** Current PI controller default gains (0.0f = unconfigured, loaded from FlashConfig/GUI) */
#define PI_ID_KP 0.0f
#define PI_ID_KI 0.0f
#define PI_IQ_KP 0.0f
#define PI_IQ_KI 0.0f

/** Speed loop output limits [A] (0.0f = unconfigured, set dynamically by FlashConfig) */
#define SPEED_LOOP_OUT_MAX 0.0f
/* Limit regenerative braking to -1.0A to prevent Overvoltage trips on power supplies */
#define SPEED_LOOP_OUT_MIN (-1.0f)

/** Default LADRC Controller Bandwidth [rad/s] */
#define LADRC_OMEGA_C_DEFAULT 35.0f

/** Default LADRC Observer Bandwidth [rad/s] */
#define LADRC_OMEGA_O_DEFAULT 120.0f

/** Default LADRC Control Gain b0 (0.0f = unconfigured, calculated after Motor ID) */
#define LADRC_B0_DEFAULT 0.0f

/*===========================================================================*/
/* SMO Observer Configuration                                                */
/*===========================================================================*/

/** SMO sliding gain - limited by L: k_slide * dt/L < few amps per step */
#define SMO_K_SLIDE 20.0f

/** SMO sigmoid bandwidth (smaller = sharper, better angle SNR but more chatter) */
#define SMO_K_SIGMOID 30.0f

/** SMO PLL bandwidth Hz */
#define SMO_PPL_CUTOFF 500.0f

/** SMO PLL integral limits [rad/s elec] - ceiling electrical speed clamp */
#define SMO_PLL_INT_MAX 25000.0f
#define SMO_PLL_INT_MIN (-25000.0f)

#define COMP_DELAY_SAMPLES 0.0f

/** Vbus IIR low-pass filter coefficient (alpha = 0.95, ~16 Hz cutoff at 1 kHz) */
#define VBUS_IIR_ALPHA 0.95f

/** ADC channel switching hysteresis (duty difference threshold to prevent jitter) */
#define SKIP_HYSTERESIS 0.03f

#define OMEGA_STF_CUTOFF 800.0f * TWO_PI
#define OMEGA_OUT_CUTOFF 300.0f * TWO_PI

/*===========================================================================*/
/* Startup Configuration                                                     */
/*===========================================================================*/

/** Alignment current [A] */
#define ALIGN_CURRENT 0.3f

/** Alignment duration [ms] */
#define ALIGN_DURATION_MS 500

/** Open-loop startup current [A] */
#define STARTUP_CURRENT 0.5f

/** Minimum continuous lock duration required before closed-loop handoff [ms] */
#define HANDOFF_LOCK_DURATION_MS 10.0f
#define HANDOFF_LOCK_SAMPLES \
    ((uint32_t)(HANDOFF_LOCK_DURATION_MS * 0.001f * (float)CONTROL_FREQUENCY))

/** Startup acceleration [rad/s^2 elec] */
#define STARTUP_ACCEL 350.0f

/** Minimum speed before switching to closed-loop [rad/s elec] */
#define STARTUP_HANDOFF_SPEED 750.0f

/** Transition blend duration from open-loop to closed-loop [ms] */
#define TRANSITION_BLEND_MS 20.0f

/** Startup timeout [ms] - set to 0 to disable */
#define STARTUP_TIMEOUT_MS 1000

/*===========================================================================*/
/* Safety / Fault Protection                                                 */
/*===========================================================================*/

/*--- Overcurrent protection ---*/
/** Overcurrent trip threshold [A].
 *  0.0f = AUTO: resolved at runtime to 1.25 x motor_max_curr
 *  (see FOC_GetOCThreshold()). Any positive value is used exactly as-is and
 *  may intentionally be set below motor_max_curr (test mode: current
 *  commands above the threshold will trip FAULT_OVERCURRENT). */
#define FAULT_OVERCURRENT_THRESHOLD 0.0f

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

/** The stall detector is a 4-layer auto-scaled system (see FOC_Safety in
 *  foc_slow_task.c): a leaky risk accumulator driven by the electromechanical
 *  power conversion ratio (eta_em), vector desynchronization (d_desync),
 *  back-EMF residual (r_bemf) and a high stall-current condition (auto
 *  0.2 x motor_max_curr, min 0.8 A). All thresholds scale with motor_max_curr,
 *  Vbus and the startup handoff speed. */

/*===========================================================================*/
/* Debug/Safety Configuration                                                */
/*===========================================================================*/

/** Enable automatic handoff from open-loop to closed-loop (0=stay in open-loop)
 */
#define ENABLE_CLOSED_LOOP_HANDOFF 1

/** Maximum runtime before auto-stop [ms] - set to 0 to disable */
#define DEBUG_RUN_TIMEOUT_MS 0

/*===========================================================================*/
/* Power-On Beep (ESC-style)                                                 */
/*===========================================================================*/

/** Enable the power-on beep sequence (0 = disabled: no beep at boot) */
#define BEEP_ENABLE 1

/** Beep tone frequencies [Hz] (ESC-style rising chime: 3 ascending tones + ready chime) */
#define BEEP_FREQ_TONE1_HZ 1480.0f /**< Tone 1 (low: ~D6/F#6) */
#define BEEP_FREQ_TONE2_HZ 1980.0f /**< Tone 2 (mid: ~B6) */
#define BEEP_FREQ_TONE3_HZ 2640.0f /**< Tone 3 (high: ~E7) */
#define BEEP_FREQ_READY_HZ 3520.0f /**< Tone 4 (final ready chime: ~A7) */

/** Beep excitation amplitude: peak d-axis voltage [V], open-loop (no current loop).
 *  The resulting peak phase current is
 *  I_pk ~= 1.5*V_AMP / (2*sqrt(Rs^2 + (2*pi*f*Ls)^2))
 *  ~= 0.5..2 A across the project motor set at 0.5 V / 1.5..3.5 kHz. */
#define BEEP_V_AMP 0.5f

/** Beep pattern timing [ms] (ESC-style: 3 ascending beeps, pause, 1 long ready beep) */
#define BEEP_SHORT_MS 100.0f /**< Duration of each short beep */
#define BEEP_GAP_MS 100.0f   /**< Gap between the short beeps */
#define BEEP_PAUSE_MS 250.0f /**< Pause before the final long beep */
#define BEEP_LONG_MS 400.0f  /**< Duration of the final "ready" beep */

/** Hard safety timeout for the whole beep sequence [ms] */
#define BEEP_MAX_TOTAL_MS 5000.0f

/*===========================================================================*/
/* Motor ID Configuration — Dual-LPF & Smart Auto-Resolution                */
/*===========================================================================*/

/** Fast filter delay target in seconds (~1.04 ms for instant ramp cut-off) */
#define ID_FAST_TAU_TARGET_S 0.00104f

/** Clamping bounds for slow filter alpha at 48 kHz (tau = 10.4 ms to 69.4 ms) */
#define ID_ALPHA_SLOW_MIN 0.0003f
#define ID_ALPHA_SLOW_MAX 0.0020f

/** Maximum expected electrical time constant*/
#define ID_MAX_MOTOR_TAU_S 0.550f

/** Minimum expected motor resistance [Ohm] */
#define ID_MIN_MOTOR_RS 0.0035f

/** 4-sigma post-filter flat envelope multiplier (99.994% noise rejection) */
#define ID_FLAT_SIGMA_MULT 4.0f

/** Minimum noise floor clamp [A] */
#define ID_NOISE_FLOOR_MIN 0.005f

/** Smart Auto-Resolution & SNR stopping criteria */
#define ID_MIN_ADC_COUNTS 150.0f   /**< Minimum ADC LSB counts for Delta_I */
#define ID_MIN_NOISE_SNR 20.0f     /**< Minimum Delta_I / noise_rms ratio */
#define ID_MIN_DELTA_VOLTAGE 0.15f /**< Minimum Delta_V [V] to overcome PWM jitter */
#define ID_MAX_DELTA_I_CALC_CAP \
    15.0f /**< Max step size cap used during analytical derivation [A] */

/** Voltage ramp rate limits [V/s] */
#define ID_RAMP_OVERSHOOT_PCT 0.05f /**< Allowed current overshoot past ramp stop (5%) */
#define ID_V_RAMP_MIN 15.0f         /**< Minimum ramp rate [V/s] */
#define ID_V_RAMP_MAX 40.0f         /**< Maximum ramp rate [V/s] */

/** Settle integration and timing */
#define ID_SETTLE_SAMPLES 360U     /**< 360-sample averaging window (7.5 ms @ 48 kHz) */
#define ID_SETTLE_HOLD_TIME_MS 15U /**< Minimum settle hold delay before averaging [ms] */
#define ID_ALIGN_DURATION_MS 150U  /**< D-axis alignment duration at I1 [ms] */

/** Hand spin BEMF Flux Identification minimum Vac threshold [V] */
#define ID_FLUX_MIN_VAC 0.3f

/*===========================================================================*/
/* Input & Throttle Defaults                                                 */
/*===========================================================================*/
#define INPUT_SOURCE_DEFAULT 1.0f /**< Default hardware: 0=NONE, 1=POT, 2=CUSTOM */
#define INPUT_MODE_DEFAULT 2.0f   /**< Default input control mode: 0=SPEED, 1=TORQUE, 2=VOLTAGE */
#define INPUT_MIN_SPEED_DEFAULT 600.0f /**< Minimum speed for throttle setpoint [rad/s elec] */
#define INPUT_MIN_CURRENT_DEFAULT 0.5f /**< Minimum current for throttle setpoint [A] */
#define INPUT_MIN_VQ_DEFAULT 0.05f     /**< Minimum voltage ratio [0.0 to 1.0] */
#define INPUT_DEADBAND_DEFAULT 0.05f   /**< Throttle deadband ratio [0.0 to 1.0] */

#define POT_ADC_MAX 4095    /**< Maximum ADC value */
#define POT_LPF_ALPHA 0.95f /**< Potentiometer LPF coefficient (~8 Hz cutoff at 1 kHz) */

/*===========================================================================*/
/* Flying Start & Active Braking Configuration                               */
/*===========================================================================*/

/** Minimum active dynamic brake duration [ms] to allow current to settle */
#define BRAKE_MIN_DURATION_MS 50U

/** Maximum fail-safe active dynamic brake timeout [ms] */
#define BRAKE_MAX_DURATION_MS 350U

/** Multiplier for adaptive noise floor threshold (4-sigma = 99.994% confidence) */
#define BRAKE_NOISE_SIGMA_MULT 4.0f

/** Lower clamp for exit current threshold [A] (well above 10-30mA ADC DC offset floor) */
#define BRAKE_EXIT_CURR_MIN 0.15f

/** Upper clamp for exit current threshold [A] */
#define BRAKE_EXIT_CURR_MAX 0.50f

/** Debounce duration [ms] confirming motor has stopped before transitioning to ALIGN */
#define BRAKE_DEBOUNCE_MS 10U

#endif /* FOC_CONFIG_H */
