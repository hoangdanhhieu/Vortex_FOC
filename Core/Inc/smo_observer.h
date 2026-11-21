/**
 * @file smo_observer.h
 * @brief Sliding Mode Observer for sensorless BLDC/PMSM control
 *
 * Improved SMO using continuous sigmoid function instead of sign()
 * for reduced chattering and smoother estimation.
 */

#ifndef SMO_OBSERVER_H
#define SMO_OBSERVER_H

#include <stdint.h>

/**
 * @brief SMO Observer structure
 */
typedef struct {
    /* Estimated currents */
    float Ialpha_est;
    float Ibeta_est;

    /* Estimated back-EMF */
    float Ealpha;
    float Ebeta;

    /* Filtered back-EMF (for angle estimation) */
    float Ealpha_flt;
    float Ebeta_flt;

    /* Estimated rotor position and speed */
    float theta_est; /**< Electrical angle [-1, 1) representing [-pi, pi) */
    float omega_est; /**< Electrical speed [rad/s] (raw PLL output) */
    float omega_out; /**< Electrical speed [rad/s] (filtered output for control loops) */

    /* Observer gains */
    float k_slide;   /**< Sliding mode gain */
    float k_sigmoid; /**< Sigmoid bandwidth parameter */

    /* PLL for angle tracking */
    float pll_kp;       /**< PLL proportional gain */
    float pll_ki;       /**< PLL integral gain */
    float pll_integral; /**< PLL integral accumulator */
    float pll_int_min;  /**< PLL integral minimum limit */
    float pll_int_max;  /**< PLL integral maximum limit */
    float direction;    /**< Active rotation direction (+1.0f or -1.0f) governed exclusively by FOC
                           State Machine */

    /* Dynamic PLL bandwidth parameters */
    float pll_cutoff_min; /**< Minimum PLL bandwidth [Hz] */
    float pll_alpha;      /**< Speed-to-bandwidth scaling factor */
    float omega_est_filt; /**< Low-pass filtered electrical speed [rad/s] */
    float omega_stf;      /**< LPF filtered speed for STF center frequency [rad/s] */

    /* 6th Harmonic Adaptive Compensator parameters */
    float Ac;            /**< Cosine 6th harmonic coefficient */
    float As;            /**< Sine 6th harmonic coefficient */
    float gamma_6th;     /**< Adaptive learning rate */
    float max_comp_norm; /**< Maximum compensation angle (normalized) */

    /* Motor parameters (cached) */
    float Rs;        /**< Phase resistance */
    float Ls;        /**< Phase inductance (nominal L0) */
    float Ls_inv;    /**< 1/Ls for faster computation */
    float sat_alpha; /**< Magnetic saturation coefficient [1/A^2] */
    float psi;       /**< Flux linkage */
    float poles;     /**< Number of pole pairs */
    /* Sample time */
    float dt; /**< Control loop period */

    /* Diagnostics & Time constants */
    float current_err_sq; /**< Squared current estimation error magnitude [A^2] */
    float theta_err;      /**< Instantaneous PLL angle tracking error [-1.0 to 1.0) */
    float bemf_mag;       /**< Instantaneous Back-EMF vector magnitude [V] */

    /* Precomputed Exact Discrete ZOH coefficients */
    float a_zoh;       /**< Discrete pole exp(-Rs * dt / Ls) */
    float b_zoh;       /**< Discrete input gain (1.0 - a_zoh) / Rs */
    float Rs_inv;      /**< Precomputed 1.0 / Rs [1/Ohm] */
    float tau_ratio_0; /**< Precomputed nominal (Rs * dt) / Ls */

    /* Precomputed constants for zero-cycle-overhead 48kHz execution */
    float dt_over_pi;      /**< Precomputed dt / PI */
    float dt_1p5_over_pi;  /**< Precomputed 1.5 * dt / PI */
    float gamma_6th_dt;    /**< Precomputed gamma_6th * dt */
    float pll_ki_dt;       /**< Precomputed pll_ki * dt */
    float omega_stf_alpha; /**< Precomputed STF center-frequency LPF alpha */
    float omega_out_alpha; /**< Precomputed speed output LPF alpha */
    float pll_cutoff_max;  /**< Maximum PLL / STF cutoff angular frequency [rad/s] */
    float omega_obs_max;   /**< Maximum observer bandwidth [rad/s] */
    float wc_base;         /**< Precomputed base cutoff frequency for STF BEMF filter [rad/s] */
    float bw_cur;
    float omega_obs;
    float phase_lag;
} SMO_Observer_t;

/* SMO_Update option flags */
#define SMO_OPT_NONE 0x00U
#define SMO_OPT_STARTUP                                                       \
    0x01U /**< Clamp PLL speed to rotation direction & track ref_omega in STF \
           */
#define SMO_OPT_HARM_COMP 0x02U /**< Enable adaptive 6th-harmonic angle error compensator */

/**
 * @brief Initialize SMO observer with default parameters
 * @param smo Pointer to SMO structure
 */
void SMO_Init(SMO_Observer_t* smo);

/**
 * @brief Reset SMO observer states
 * @param smo Pointer to SMO structure
 */
void SMO_Reset(SMO_Observer_t* smo);

/**
 * @brief Seed SMO observer speed, angle, and current states (e.g. exiting zero-crossing bridge)
 * @param smo Pointer to SMO structure
 * @param omega_init Initial electrical speed in rad/s
 * @param theta_norm Initial electrical angle in [-1, 1) representing [-pi, pi)
 * @param Ialpha_init Initial alpha current estimate [A]
 * @param Ibeta_init Initial beta current estimate [A]
 */
void SMO_ResetStatesWithAngle(SMO_Observer_t* smo, float omega_init, float theta_norm,
                              float Ialpha_init, float Ibeta_init);

/**
 * @brief Seed SMO PLL/speed states and gains when entering Flying Start from Detect
 * @param smo Pointer to SMO structure
 * @param theta_norm Initial electrical angle in [-1, 1) representing [-pi, pi)
 * @param omega_elec Estimated electrical speed [rad/s]
 * @param dir Rotation direction (+1.0f or -1.0f)
 * @param vbus Current DC bus voltage [V]
 */
void SMO_SeedFlyingStart(SMO_Observer_t* smo, float theta_norm, float omega_elec, float dir,
                         float vbus);

/**
 * @brief Seed SMO current estimate onto the sliding surface from filtered BEMF (Flying Start ->
 * RUN)
 * @param smo Pointer to SMO structure
 * @param Ialpha_init Measured/filtered alpha current [A]
 * @param Ibeta_init Measured/filtered beta current [A]
 */
void SMO_SeedCurrentFromBEMF(SMO_Observer_t* smo, float Ialpha_init, float Ibeta_init);

/**
 * @brief Set SMO active rotation direction
 * @param smo Pointer to SMO structure
 * @param dir Direction (+1.0f for forward, -1.0f for reverse)
 */
void SMO_SetDirection(SMO_Observer_t* smo, float dir);

/**
 * @brief Update SMO observer with new voltage and current measurements
 * @param smo Pointer to SMO structure
 * @param Valpha Alpha voltage command [V]
 * @param Vbeta Beta voltage command [V]
 * @param Ialpha Measured alpha current [A]
 * @param Ibeta Measured beta current [A]
 * @param l_app_ratio Apparent inductance saturation ratio L_app(I)/L0 in (0, 1]
 * @param opts Bitmask of SMO_OPT_* flags
 * @param ref_omega Reference electrical speed [rad/s] used when SMO_OPT_STARTUP is set
 */
void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha, float Ibeta,
                float l_app_ratio, uint8_t opts, float ref_omega);

/**
 * @brief 1 kHz SMO adaptation task (adjust pll_kp, pll_ki, k_slide, k_sigmoid, and bemf_mag).
 * @param smo Pointer to SMO structure
 * @param is_active Non-zero when observer adaptation should run
 * @param is_startup Non-zero when open-loop startup reference speed should be used
 * @param startup_omega Open-loop startup electrical speed [rad/s]
 * @param vbus Current DC bus voltage [V]
 */
void SMO_SlowTask(SMO_Observer_t* smo, uint8_t is_active, uint8_t is_startup, float startup_omega,
                  float vbus);

#include "foc.h"

/**
 * @brief Get estimated electrical angle for Park transform (no hardware delay compensation)
 * @param smo Pointer to SMO structure
 * @return Electrical angle in [-1, 1) representing [-pi, pi)
 */
CCMRAM_FUNC static inline float SMO_GetParkAngle(const SMO_Observer_t* smo) {
    return normalize_angle_norm(smo->theta_est + smo->phase_lag);
}

/**
 * @brief Get estimated electrical angle for PWM generation (compensated for hardware delay)
 * @param smo Pointer to SMO structure
 * @return Electrical angle in [-1, 1) representing [-pi, pi)
 */
CCMRAM_FUNC static inline float SMO_GetPWMAngle(const SMO_Observer_t* smo) {
    float theta_advance = smo->omega_est * smo->dt_1p5_over_pi;
    return normalize_angle_norm(smo->theta_est + theta_advance + smo->phase_lag);
}

/**
 * @brief Get estimated electrical speed
 * @param smo Pointer to SMO structure
 * @return Electrical speed in rad/s
 */
CCMRAM_FUNC static inline float SMO_GetSpeed(const SMO_Observer_t* smo) {
    return smo->omega_out;
}

/**
 * @brief Set dynamic motor parameters at runtime
 * @param smo Pointer to SMO structure
 * @param Rs Phase resistance [Ohm]
 * @param Ls Phase inductance [H]
 * @param sat_alpha Magnetic saturation coefficient [1/A^2]
 * @param flux_linkage Flux linkage [Wb]
 * @param poles Number of pole pairs
 * @param max_speed_elec_rad Maximum motor speed in electrical rad/s
 */
void SMO_SetMotorParams(SMO_Observer_t* smo, float Rs, float Ls, float sat_alpha,
                        float flux_linkage, float poles, float max_speed_elec_rad, float bw_cur);

/**
 * @brief Feed external BEMF directly into the shared STF + PLL track loop
 *        (bypasses the current observer). Used during Flying Start.
 *
 *        The BEMF is passed through the same Self-Tuning Filter as the
 *        internal SMO path, then tracked by SMO_PLL_Track with the
 *        0.3 dt ADC sampling delay compensation and without the 6th
 *        harmonic compensation blocks.
 * @param smo Pointer to SMO structure
 * @param Ealpha Alpha-axis BEMF voltage [V]
 * @param Ebeta Beta-axis BEMF voltage [V]
 */
void SMO_FeedBEMF(SMO_Observer_t* smo, float Ealpha, float Ebeta);

/**
 * @brief Set timing (sample time dt) for SMO observer
 * @param smo Pointer to SMO structure
 * @param dt Control period in seconds
 */
void SMO_SetTiming(SMO_Observer_t* smo, float dt);

#endif /* SMO_OBSERVER_H */
