#include "foc.h"

#include "cordic_math.h"
#include "foc_config.h"
#include "foc_state_machine.h"

static float s_cos_th = 1.0f;
static float s_sin_th = 0.0f;
/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

NOINLINE_FUNC CCMRAM_FUNC void svpwm_calculate(float theta) {
    float Vbus_inv = g_foc.data.Vbus_inv;

    float v_max_limit = g_foc.v_max_scale;
    float v_max_sq = v_max_limit * v_max_limit;

    float vd_norm = g_foc.data.Vd * Vbus_inv;
    float vq_norm = g_foc.data.Vq * Vbus_inv;

    float v_sq = vd_norm * vd_norm + vq_norm * vq_norm;
    if (v_sq > v_max_sq) {
        vd_norm = saturatef(vd_norm, v_max_limit);
        float vq_max_sq = v_max_sq - vd_norm * vd_norm;
        float vq_max = (vq_max_sq > 0.0f) ? sqrtf(vq_max_sq) : 0.0f;
        vq_norm = saturatef(vq_norm, vq_max);
        g_foc.data.Vd = vd_norm * g_foc.data.Vbus;
        g_foc.data.Vq = vq_norm * g_foc.data.Vbus;
    }

    cordic_sincos(theta, &s_cos_th, &s_sin_th);
    float va_norm, vb_norm;
    inverse_park_transform(vd_norm, vq_norm, s_cos_th, s_sin_th, &va_norm, &vb_norm);

    /* Calculate phase voltages using inverse Clarke */
    float Va = va_norm;
    float Vb = -0.5f * va_norm + (SQRT3 * 0.5f) * vb_norm;
    float Vc = -0.5f * va_norm - (SQRT3 * 0.5f) * vb_norm;

    /* Find min and max for midpoint clamping (SVPWM) */
    float Vmin = Va < Vb ? Va : Vb;
    Vmin = Vmin < Vc ? Vmin : Vc;

    float Vmax = Va > Vb ? Va : Vb;
    Vmax = Vmax > Vc ? Vmax : Vc;

    /* Midpoint offset for centered PWM (symmetric SVPWM) */
    float Voffset = (Vmax + Vmin) * 0.5f;

    /* Calculate initial duty cycles (0.5 centered) */
    float da = (Va - Voffset) + 0.5f;
    float db = (Vb - Voffset) + 0.5f;
    float dc = (Vc - Voffset) + 0.5f;

    /* Dynamic downward zero-sequence shift: keeps 0.5-centered 7-segment SVPWM at low/mid
     * modulation, and shifts all 3 phases down equally when d_max > max_duty so line-to-line
     * voltage reaches (max_duty - min_duty) without ever violating max_duty. */
    float d_max = (da > db) ? da : db;
    if (dc > d_max) d_max = dc;
    if (d_max > g_foc.max_duty) {
        float shift = d_max - g_foc.max_duty;
        da -= shift;
        db -= shift;
        dc -= shift;
    }

    g_foc.data.duty_a = da;
    g_foc.data.duty_b = db;
    g_foc.data.duty_c = dc;

    g_foc.data.Valpha = va_norm * g_foc.data.Vbus;
    g_foc.data.Vbeta = vb_norm * g_foc.data.Vbus;
}

/**
 * @brief Apply centralized deadtime compensation to duty cycles.
 */

static inline float deadtime_compensate_phase(float duty, float I_flt, float inv_i_th,
                                              float dt_comp, float max_limit) {
    float x = clampf(I_flt * inv_i_th, -1.0f, 1.0f);
    float factor = x * (1.5f - 0.5f * x * x);
    return clampf(duty + dt_comp * factor, 0.0f, max_limit);
}

NOINLINE_FUNC CCMRAM_FUNC void foc_apply_deadtime_compensation(float* out_a, float* out_b,
                                                               float* out_c) {
    float dt_comp = g_foc.deadtime_duty;
    if (dt_comp <= 0.0001f) return;

    float current_max_duty = *out_a;
    if (*out_b > current_max_duty) current_max_duty = *out_b;
    if (*out_c > current_max_duty) current_max_duty = *out_c;
    float max_limit = g_foc.max_duty;
    float fade_start = max_limit * 0.90f;
    if (current_max_duty > fade_start) {
        float fade_factor = (max_limit - current_max_duty) / (max_limit - fade_start);
        dt_comp *= (fade_factor > 0.0f) ? fade_factor : 0.0f;
    }
    if (dt_comp <= 0.0001f) return;

    float Ialpha_pred, Ibeta_pred;
    inverse_park_transform(g_foc.data.Id, g_foc.data.Iq, s_cos_th, s_sin_th, &Ialpha_pred,
                           &Ibeta_pred);
    float Ia, Ib, Ic;
    inverse_clarke_transform(Ialpha_pred, Ibeta_pred, &Ia, &Ib, &Ic);

    float inv_i_th = g_foc.data.inv_i_th;
    *out_a = deadtime_compensate_phase(*out_a, Ia, inv_i_th, dt_comp, max_limit);
    *out_b = deadtime_compensate_phase(*out_b, Ib, inv_i_th, dt_comp, max_limit);
    *out_c = deadtime_compensate_phase(*out_c, Ic, inv_i_th, dt_comp, max_limit);
}
