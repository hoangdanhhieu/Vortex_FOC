#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#include "foc_config.h"
#include "foc_hardware.h"

/*===========================================================================*/
/* Clarke/Park Transforms                                                    */
/*===========================================================================*/

CCMRAM_FUNC static inline void clarke_transform(float Ia, float Ib, float Ic, float* alpha,
                                                float* beta) {
    *alpha = Ia;
    *beta = (Ib - Ic) * SQRT3_INV;
}

/**
 * @brief Inverse Clarke transform: Ialpha,Ibeta -> Ia,Ib,Ic
 */
CCMRAM_FUNC static inline void inverse_clarke_transform(float alpha, float beta, float* Ia,
                                                        float* Ib, float* Ic) {
    *Ia = alpha;
    *Ib = -0.5f * alpha + 0.8660254f * beta;
    *Ic = -(*Ia + *Ib);
}

/**
 * @brief Park transform: Ialpha,Ibeta -> Id,Iq
 */
CCMRAM_FUNC static inline void park_transform(float Ialpha, float Ibeta, float cos_th, float sin_th,
                                              float* Id, float* Iq) {
    *Id = Ialpha * cos_th + Ibeta * sin_th;
    *Iq = -Ialpha * sin_th + Ibeta * cos_th;
}

/**
 * @brief Inverse Park transform: Vd,Vq -> Valpha,Vbeta
 */
CCMRAM_FUNC static inline void inverse_park_transform(float Vd, float Vq, float cos_th,
                                                      float sin_th, float* Valpha, float* Vbeta) {
    *Valpha = Vd * cos_th - Vq * sin_th;
    *Vbeta = Vd * sin_th + Vq * cos_th;
}

/**
 * @brief Normalize angle to [-1, 1) range (corresponds to [-pi, pi))
 */
CCMRAM_FUNC static inline float normalize_angle_norm(float angle) {
    float y = (angle + 1.0f) * 0.5f + 64.0f;
    float flr = (float)((int32_t)y) - 64.0f;
    return angle - 2.0f * flr;
}

/**
 * @brief Exact Discrete Rotational STF step using explicit decay and input blend coefficients.
 */
CCMRAM_FUNC static inline void stf_filter_step_coeff(float in_alpha, float in_beta,
                                                     float* flt_alpha, float* flt_beta, float decay,
                                                     float input_gain, float omega, float dt) {
    float theta_step = omega * dt;
    float th_sq = theta_step * theta_step;
    float cos_step = 1.0f - 0.5f * th_sq + (0.04166667f) * th_sq * th_sq;
    float sin_step = theta_step * (1.0f - 0.16666667f * th_sq);
    float prev_a = *flt_alpha;
    float prev_b = *flt_beta;
    float rot_a = cos_step * prev_a - sin_step * prev_b;
    float rot_b = sin_step * prev_a + cos_step * prev_b;

    *flt_alpha = decay * rot_a + input_gain * in_alpha;
    *flt_beta = decay * rot_b + input_gain * in_beta;
}

/**
 * @brief Exact Discrete Rotational Self-Tuning Filter (STF) step in alpha-beta frame
 * @details Filters input signals in stationary alpha-beta frame while tracking the fundamental
 * frequency:
 *          1. Rotates previous filtered state by theta_step = omega * dt.
 *          2. Applies continuous decay exp(-wc * dt) via 3rd-order Taylor expansion.
 *          3. Blends new input with gain (1.0 - decay).
 *          Optimized for 48kHz ISR execution without floating-point division (vdiv.f32).
 * @param in_alpha  Alpha input signal
 * @param in_beta   Beta input signal
 * @param flt_alpha Pointer to filtered alpha state variable (updated in-place)
 * @param flt_beta  Pointer to filtered beta state variable (updated in-place)
 * @param wc        Cutoff angular frequency [rad/s]
 * @param omega     Center tracking angular frequency [rad/s]
 * @param dt        Sample period [s]
 */
CCMRAM_FUNC static inline void stf_filter_step(float in_alpha, float in_beta, float* flt_alpha,
                                               float* flt_beta, float wc, float omega, float dt) {
    float a = wc * dt;
    float decay = 1.0f - a * (1.0f - a * (0.5f - a * 0.16666667f));
    decay = clampf(decay, 0.0f, 1.0f);
    float input_gain = 1.0f - decay;
    stf_filter_step_coeff(in_alpha, in_beta, flt_alpha, flt_beta, decay, input_gain, omega, dt);
}

/*===========================================================================*/
/* Space Vector PWM & Deadtime Compensation                                  */
/*===========================================================================*/

/**
 * @brief Convert normalized stationary voltages (io_va_norm, io_vb_norm) to 3-phase SVPWM duties.
 *        Zone 1 (ovm_en == 0): keeps all 3 phases strictly in [0.0, max_duty].
 *        Zone 2 (ovm_en != 0, Zero-Forbidden-Band DPWM1): when span > max_duty and the top-to-mid
 *        gap allows clamping the highest phase flat to 1.000 (CCR = ARR+1, zero switching edges)
 *        while keeping the middle phase <= max_duty and min phase >= 0.0, clamps top phase to
 *        1.000; otherwise falls back radially into [0.0, max_duty]. Strictly guarantees ZERO
 *        phases ever lie in the forbidden switching band (max_duty, 1.000).
 *        Scales (*io_va_norm, *io_vb_norm) in-place when radial scaling occurs.
 * @return Applied radial scale factor (1.0f if unscaled, < 1.0f when radially clipped)
 */
CCMRAM_FUNC static inline float foc_svpwm_ab(float* io_va_norm, float* io_vb_norm, float max_duty,
                                             uint8_t ovm_en, float* out_da, float* out_db,
                                             float* out_dc) {
    float va_n = *io_va_norm;
    float vb_n = *io_vb_norm;

    float Va = va_n;
    float Vb = -0.5f * va_n + (SQRT3 * 0.5f) * vb_n;
    float Vc = -0.5f * va_n - (SQRT3 * 0.5f) * vb_n;

    float Vmin = (Va < Vb) ? Va : Vb;
    if (Vc < Vmin) Vmin = Vc;

    float Vmax = (Va > Vb) ? Va : Vb;
    if (Vc > Vmax) Vmax = Vc;

    /* Va + Vb + Vc == 0 identically -> zero-branch mid-phase extraction */
    float Vmid = -(Vmax + Vmin);
    float span = Vmax - Vmin;
    float scale = 1.0f;
    uint8_t clamp_top = 0U;

    if (span > max_duty && span > 1e-6f) {
        if (ovm_en) {
            float s2 = (span > 1.0f) ? (1.0f / span) : 1.0f;
            float top_gap = Vmax - Vmid;
            float forbidden_gap = 1.0f - max_duty;
            if (s2 * top_gap >= forbidden_gap) {
                scale = s2;
                clamp_top = 1U;
            } else {
                scale = max_duty / span;
            }
        } else {
            scale = max_duty / span;
        }
        if (scale < 1.0f) {
            Va *= scale;
            Vb *= scale;
            Vc *= scale;
            Vmin *= scale;
            Vmax *= scale;
            *io_va_norm = va_n * scale;
            *io_vb_norm = vb_n * scale;
        }
    }

    float shift;
    if (clamp_top) {
        /* DPWM1 flat-top clamp: d_max == 1.000, d_mid <= max_duty, d_min >= 0.0 */
        shift = 1.0f - Vmax;
    } else {
        /* Standard centered SVPWM shifted if needed to stay strictly in [0.0, max_duty] */
        shift = 0.5f - 0.5f * (Vmax + Vmin);
        if (Vmax + shift > max_duty) {
            shift = max_duty - Vmax;
        } else if (Vmin + shift < 0.0f) {
            shift = -Vmin;
        }
    }

    float da = Va + shift;
    float db = Vb + shift;
    float dc = Vc + shift;

    if (clamp_top) {
        *out_da = (da >= 0.9999f) ? 1.0f : clampf(da, 0.0f, max_duty);
        *out_db = (db >= 0.9999f) ? 1.0f : clampf(db, 0.0f, max_duty);
        *out_dc = (dc >= 0.9999f) ? 1.0f : clampf(dc, 0.0f, max_duty);
    } else {
        *out_da = clampf(da, 0.0f, max_duty);
        *out_db = clampf(db, 0.0f, max_duty);
        *out_dc = clampf(dc, 0.0f, max_duty);
    }
    return scale;
}

/**
 * @brief Inverse Park transform and 3-phase SVPWM duty calculation from normalized (Vd, Vq).
 */
CCMRAM_FUNC static inline float foc_svpwm_dq(float vd_norm, float vq_norm, float cos_th,
                                             float sin_th, float max_duty, uint8_t ovm_en,
                                             float* out_va_norm, float* out_vb_norm, float* out_da,
                                             float* out_db, float* out_dc) {
    inverse_park_transform(vd_norm, vq_norm, cos_th, sin_th, out_va_norm, out_vb_norm);
    return foc_svpwm_ab(out_va_norm, out_vb_norm, max_duty, ovm_en, out_da, out_db, out_dc);
}

/**
 * @brief Smooth cubic deadtime compensation with per-phase lower/upper boundary fade & clamping.
 */
CCMRAM_FUNC static inline float deadtime_compensate_phase(float duty, float I_flt, float inv_i_th,
                                                          float dt_comp, float fade_span,
                                                          float fade_inv, float max_duty) {
    if (duty >= 0.9999f) {
        return 1.0f;
    }
    float margin_hi = max_duty - duty;
    float margin_lo = duty;
    float margin = (margin_hi < margin_lo) ? margin_hi : margin_lo;
    if (margin <= 0.0f) {
        return clampf(duty, 0.0f, max_duty);
    }
    if (margin < fade_span) {
        dt_comp *= margin * fade_inv;
    }

    float x = I_flt * inv_i_th;
    float factor;
    if (x >= 1.0f) {
        factor = 1.0f;
    } else if (x <= -1.0f) {
        factor = -1.0f;
    } else {
        factor = x * (1.5f - 0.5f * x * x);
    }
    return clampf(duty + dt_comp * factor, 0.0f, max_duty);
}

/**
 * @brief Apply per-phase fade-clamped deadtime compensation to 3-phase duty cycles.
 *        Flat-top 1.000 phases remain untouched at 1.0f; switching phases stay strictly in
 *        [0.0f, max_duty] so zero duties ever enter the forbidden switching band.
 */
CCMRAM_FUNC static inline void foc_deadtime_compensate_dq(float id, float iq, float cos_th,
                                                          float sin_th, float dt_comp,
                                                          float fade_span, float fade_inv,
                                                          float max_duty, float inv_i_th, float* da,
                                                          float* db, float* dc) {
    if (dt_comp <= 0.0001f) return;

    float Ialpha_pred, Ibeta_pred;
    inverse_park_transform(id, iq, cos_th, sin_th, &Ialpha_pred, &Ibeta_pred);
    float Ia, Ib, Ic;
    inverse_clarke_transform(Ialpha_pred, Ibeta_pred, &Ia, &Ib, &Ic);

    *da = deadtime_compensate_phase(*da, Ia, inv_i_th, dt_comp, fade_span, fade_inv, max_duty);
    *db = deadtime_compensate_phase(*db, Ib, inv_i_th, dt_comp, fade_span, fade_inv, max_duty);
    *dc = deadtime_compensate_phase(*dc, Ic, inv_i_th, dt_comp, fade_span, fade_inv, max_duty);
}

/**
 * @brief SVPWM calculation - converts Vd,Vq to duty cycles (for Align/Startup/Beep/ID)
 */
void svpwm_calculate(float theta);

/**
 * @brief Set PWM duty cycles to TIM1
 * @param duty_a Phase A duty (0.0 to 1.0)
 * @param duty_b Phase B duty (0.0 to 1.0)
 * @param duty_c Phase C duty (0.0 to 1.0)
 * @param arr_f  Precomputed float copy of TIM1->ARR
 */
CCMRAM_FUNC static inline void foc_set_pwm_duty(float duty_a, float duty_b, float duty_c,
                                                float arr_f) {
    FOC_HW_SetPWMDuty(duty_a, duty_b, duty_c, arr_f);
}

/*===========================================================================*/
/* ADC Conversion Functions                                                  */
/*===========================================================================*/

/**
 * @brief Convert ADC reading to Vbus voltage
 * @param adc_value Raw ADC value (12-bit)
 * @return DC bus voltage in Volts
 */
CCMRAM_FUNC static inline float foc_adc_to_vbus(uint16_t adc_value) {
    return (float)adc_value * ADC_TO_VBUS;
}

/**
 * @brief Convert ADC reading to Phase voltage using 3-resistor bias divider formula (differential
 * with offset)
 * @param adc_value Raw ADC value (12-bit)
 * @param adc_offset Offset ADC value calibrated at zero phase voltage
 * @return Phase voltage in Volts (can be positive or negative)
 */
CCMRAM_FUNC static inline float foc_adc_to_vphase(uint16_t adc_value, uint16_t adc_offset) {
    float adc_diff = (float)adc_value - (float)adc_offset;
    return adc_diff * ADC_Vref * (PHASE_VOLTAGE_GAIN / (float)ADC_RESOLUTION);
}

/**
 * @brief Convert ADC reading to absolute Phase voltage (without offset subtraction)
 * @param adc_value Raw ADC value (12-bit)
 * @return Absolute Phase voltage in Volts (referenced to MCU GND)
 */
CCMRAM_FUNC static inline float foc_adc_to_vphase_absolute(uint16_t adc_value) {
    float v_adc = (float)adc_value * (ADC_Vref / (float)ADC_RESOLUTION);
    return v_adc * PHASE_VOLTAGE_GAIN - ADC_Vref * PHASE_VOLTAGE_OFFSET_FACTOR;
}

#endif /* FOC_H */