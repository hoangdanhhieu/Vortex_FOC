/**
 * @file smo_observer.h
 * @brief Sliding Mode Observer for sensorless BLDC/PMSM control
 *
 * Improved SMO using continuous sigmoid function instead of sign()
 * for reduced chattering and smoother estimation.
 */

#ifndef SMO_OBSERVER_H
#define SMO_OBSERVER_H

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
    float omega_est; /**< Electrical speed [rad/s] */

    /* Observer gains */
    float k_slide;   /**< Sliding mode gain */
    float k_sigmoid; /**< Sigmoid bandwidth parameter */

    /* Low-pass filter coefficient */
    float lpf_coeff; /**< LPF coefficient for BEMF filtering */
    float tau;       /**< Filter time constant for phase compensation */
    /* PLL for angle tracking */
    float pll_kp;       /**< PLL proportional gain */
    float pll_ki;       /**< PLL integral gain */
    float pll_integral; /**< PLL integral accumulator */
    float pll_int_min;  /**< PLL integral minimum limit */
    float pll_int_max;  /**< PLL integral maximum limit */

    /* Motor parameters (cached) */
    float Rs;     /**< Phase resistance */
    float Ls;     /**< Phase inductance */
    float Ls_inv; /**< 1/Ls for faster computation */
    float psi;    /**< Flux linkage */
    float poles;  /**< Number of pole pairs */
    /* Sample time */
    float dt; /**< Control loop period */
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
 * @brief Update SMO observer with new voltage and current measurements
 * @param smo Pointer to SMO structure
 * @param Valpha Alpha voltage command [V]
 * @param Vbeta Beta voltage command [V]
 * @param Ialpha Measured alpha current [A]
 * @param Ibeta Measured beta current [A]
 */
void SMO_Update(SMO_Observer_t* smo, float Valpha, float Vbeta, float Ialpha, float Ibeta);

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
 * @param flux_linkage Flux linkage [Wb]
 * @param poles Number of pole pairs
 */
void SMO_SetMotorParams(SMO_Observer_t* smo, float Rs, float Ls, float flux_linkage, float poles);

/**
 * @brief Set observer gains at runtime
 * @param smo Pointer to SMO structure
 * @param k_slide Sliding gain
 * @param k_sigmoid Sigmoid bandwidth
 */
void SMO_SetGains(SMO_Observer_t* smo, float k_slide, float k_sigmoid);

/**
 * @brief Set observer filter parameters at runtime
 * @param smo Pointer to SMO structure
 * @param pll_cutoff_hz PLL bandwidth frequency [Hz]
 */
void SMO_SetFilterParams(SMO_Observer_t* smo, float pll_cutoff_hz);

/**
 * @brief Feed external BEMF directly into PLL (bypass current observer)
 * @param smo Pointer to SMO structure
 * @param Ealpha Alpha-axis BEMF voltage [V]
 * @param Ebeta Beta-axis BEMF voltage [V]
 */
void SMO_FeedBEMF(SMO_Observer_t* smo, float Ealpha, float Ebeta);

#endif /* SMO_OBSERVER_H */
