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

    /* Dynamic PLL bandwidth parameters */
    float pll_cutoff_min; /**< Minimum PLL bandwidth [Hz] */
    float pll_alpha;      /**< Speed-to-bandwidth scaling factor */
    float omega_est_filt; /**< Low-pass filtered electrical speed [rad/s] */
    float omega_stf;      /**< 48kHz LPF filtered speed for STF center frequency [rad/s] */

    /* 6th Harmonic Adaptive Compensator parameters */
    float Ac;                     /**< Cosine 6th harmonic coefficient */
    float As;                     /**< Sine 6th harmonic coefficient */
    float gamma_6th;              /**< Adaptive learning rate */
    float max_comp_norm;          /**< Maximum compensation angle (normalized) */
    uint8_t enable_harmonic_comp; /**< Flag to enable/disable 6th harmonic compensator */

    /* Motor parameters (cached) */
    float Rs;        /**< Phase resistance */
    float Ls;        /**< Phase inductance (nominal L0) */
    float Ls_inv;    /**< 1/Ls for faster computation */
    float sat_alpha; /**< Magnetic saturation coefficient [1/A^2] */
    float psi;       /**< Flux linkage */
    float poles;     /**< Number of pole pairs */
    float min_omega; /**< Minimum omega for STF decay (from motor_min_spd) [rad/s] */
    /* Sample time */
    float dt; /**< Control loop period */

    /* Diagnostics & Time constants */
    float current_err_sq; /**< Squared current estimation error magnitude [A^2] */
    float tau_current;    /**< Dynamic current observer time constant [s] */

    /* Precomputed Implicit Backward Euler coefficients */
    float Req;        /**< Dynamic sliding equivalent resistance [Ohm] */
    float dt_over_Ls; /**< Precomputed dt * Ls_inv */
    float denom_inv;  /**< Precomputed 1.0 / (1.0 + Rs * dt_over_Ls) */

    /* Dynamic load current magnitude & saturation */
    float I_mag;   /**< Measured phase current magnitude [A] */
    float l_ratio; /**< Real-time L(I)/L0 saturation ratio (shared with current loop) */
} SMO_Observer_t;

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
 * @brief Seed SMO observer speed states at handoff (open-loop to closed-loop)
 * @param smo Pointer to SMO structure
 * @param omega_init Initial electrical speed in rad/s
 */
void SMO_ResetStates(SMO_Observer_t* smo, float omega_init);

/**
 * @brief Update SMO observer with new voltage and current measurements
 * @param smo Pointer to SMO structure
 * @param Valpha Alpha voltage command [V]
 * @param Vbeta Beta voltage command [V]
 * @param Ialpha Measured alpha current [A]
 * @param Ibeta Measured beta current [A]
 */
void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha, float Ibeta);

/**
 * @brief 1 kHz SMO adaptation task (adjust pll_kp, pll_ki, k_slide and k_sigmoid).
 */
void SMO_SlowTask(SMO_Observer_t* smo);

/**
 * @brief Get estimated electrical angle
 * @param smo Pointer to SMO structure
 * @return Electrical angle in [-1, 1) representing [-pi, pi)
 */
float SMO_GetAngle(SMO_Observer_t* smo);

/**
 * @brief Get estimated electrical speed
 * @param smo Pointer to SMO structure
 * @return Electrical speed in rad/s
 */
float SMO_GetSpeed(SMO_Observer_t* smo);

/**
 * @brief Get estimated mechanical speed in RPM
 * @param smo Pointer to SMO structure
 * @return Mechanical speed in RPM
 */
float SMO_GetSpeedRPM(SMO_Observer_t* smo);

/**
 * @brief Set dynamic motor parameters at runtime
 * @param smo Pointer to SMO structure
 * @param Rs Phase resistance [Ohm]
 * @param Ls Phase inductance [H]
 * @param sat_alpha Magnetic saturation coefficient [1/A^2]
 * @param flux_linkage Flux linkage [Wb]
 * @param poles Number of pole pairs
 * @param max_speed_rpm Maximum motor speed in RPM
 * @param min_speed_rpm Minimum motor speed in RPM
 */
void SMO_SetMotorParams(SMO_Observer_t* smo, float Rs, float Ls, float sat_alpha, float flux_linkage,
                        float poles, float max_speed_rpm, float min_speed_rpm);

/**
 * @brief Feed external BEMF directly into PLL (bypass current observer)
 * @param smo Pointer to SMO structure
 * @param Ealpha Alpha-axis BEMF voltage [V]
 * @param Ebeta Beta-axis BEMF voltage [V]
 */
void SMO_FeedBEMF(SMO_Observer_t* smo, float Ealpha, float Ebeta);

#endif /* SMO_OBSERVER_H */
