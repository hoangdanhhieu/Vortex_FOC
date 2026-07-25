#include "foc.h"

#include <main.h>

#include "cordic_math.h"
#include "foc_config.h"
#include "foc_state_machine.h"

/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

CCMRAM_FUNC void svpwm_calculate(void) {
    float Vbus_inv = g_foc.data.Vbus_inv;

    /* Normalize voltages to Vbus */
    /* Vd-Priority Voltage Vector Limiting:
     * Preserve 100% of Vd voltage (Id=0A control) to prevent magnetic axis angle slip,
     * and allocate all remaining voltage margin exclusively to Vq. */
    float max_duty = g_foc.max_duty;
    float v_max_limit = SQRT3_INV * 2.0f * (max_duty - 0.5f);
    if (v_max_limit < 0.0f) v_max_limit = 0.0f;

    float vd_norm = g_foc.data.Vd * Vbus_inv;
    float vq_norm = g_foc.data.Vq * Vbus_inv;

    /* Priority 1: Preserve Vd up to v_max_limit */
    vd_norm = saturatef(vd_norm, v_max_limit);

    /* Priority 2: Allocate remaining voltage margin to Vq */
    float vq_max_sq = v_max_limit * v_max_limit - vd_norm * vd_norm;
    float vq_max = (vq_max_sq > 0.0f) ? sqrtf(vq_max_sq) : 0.0f;
    vq_norm = saturatef(vq_norm, vq_max);

    /* Update real Vd and Vq */
    g_foc.data.Vd = vd_norm * g_foc.data.Vbus;
    g_foc.data.Vq = vq_norm * g_foc.data.Vbus;

    /* Recompute real Valpha and Vbeta from prioritized (Vd, Vq) */
    float cos_th, sin_th;
    cordic_sincos(g_foc.data.theta_elec, &cos_th, &sin_th);
    inverse_park_transform(g_foc.data.Vd, g_foc.data.Vq, cos_th, sin_th, &g_foc.data.Valpha,
                           &g_foc.data.Vbeta);

    float va_norm = g_foc.data.Valpha * Vbus_inv;
    float vb_norm = g_foc.data.Vbeta * Vbus_inv;

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

    /* Clamp duty cycles to valid range [0, max_duty] */
    g_foc.data.duty_a = da < 0.0f ? 0.0f : (da > max_duty ? max_duty : da);
    g_foc.data.duty_b = db < 0.0f ? 0.0f : (db > max_duty ? max_duty : db);
    g_foc.data.duty_c = dc < 0.0f ? 0.0f : (dc > max_duty ? max_duty : dc);
}
