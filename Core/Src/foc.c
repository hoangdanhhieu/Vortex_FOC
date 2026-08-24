#include "foc.h"

#include <main.h>

#include "cordic_math.h"
#include "foc_config.h"
#include "foc_state_machine.h"

/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

CCMRAM_FUNC void svpwm_calculate(float theta) {
    float Vbus_inv = g_foc.data.Vbus_inv;

    /* Normalize voltages to Vbus */
    /* Vd-Priority Voltage Vector Limiting:
     * Preserve 100% of Vd voltage (Id=0A control) to prevent magnetic axis angle slip,
     * and allocate all remaining voltage margin exclusively to Vq.
     *
     * v_max_limit is based on MAX_DUTY_HIGH (not max_duty) because the highest-duty
     * phase is always reconstructed via Kirchhoff — it does not need to be sampled.
     * This allows a larger voltage circle and more available torque. */
    float max_duty = g_foc.max_duty;
    float v_max_limit = SQRT3_INV * 2.0f * (MAX_DUTY_HIGH - 0.5f);
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
    cordic_sincos(theta, &cos_th, &sin_th);
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

    /* Selective Phase Clamping for 3-shunt low-side current sensing:
     *
     * Sort phases descending by duty cycle:
     *   ph[0] = highest duty → allowed up to MAX_DUTY_HIGH (0.95)
     *                          Current CANNOT be sampled (off-time too short)
     *                          RECONSTRUCTED each ISR via Kirchhoff in foc_reconstruct_currents()
     *   ph[1] = middle duty  → clamped to max_duty (sampling-safe)
     *                          If in blind zone (both ph[0] and ph[1] were above max_duty),
     *                          clamping ph[1] down creates a measurement window — always measurable
     *   ph[2] = lowest duty  → always below max_duty — always measurable
     *
     * This guarantees exactly 2 measurable phases at all angles and modulation indices,
     * eliminating the need for current hold in the blind zone near sector boundaries. */
    float* ph[3] = {&da, &db, &dc};
    if (*ph[0] < *ph[1]) {
        float* t = ph[0];
        ph[0] = ph[1];
        ph[1] = t;
    }
    if (*ph[0] < *ph[2]) {
        float* t = ph[0];
        ph[0] = ph[2];
        ph[2] = t;
    }
    if (*ph[1] < *ph[2]) {
        float* t = ph[1];
        ph[1] = ph[2];
        ph[2] = t;
    }
    // const float min_diff = 0.0f;
    // if (g_foc.status.state == FOC_STATE_RUN && *ph[0] > 0.5f) {
    //     if ((*ph[0] - *ph[1]) < min_diff) {
    //         *ph[1] = clampf(*ph[0] - min_diff, 0.0f, max_duty);
    //     }

    //     if ((*ph[1] - *ph[2]) < min_diff) {
    //         *ph[2] = clampf(*ph[1] - min_diff, 0.0f, max_duty);
    //     }
    // }

    *ph[0] = clampf(*ph[0], 0.0f, MAX_DUTY_HIGH); /* highest: up to 0.95, reconstructed  */
    *ph[1] = clampf(*ph[1], 0.0f, max_duty);      /* middle:  forced ≤ max_duty, measured */
    *ph[2] = clampf(*ph[2], 0.0f, max_duty);      /* lowest:  always ≤ max_duty, measured */

    g_foc.data.duty_a = da;
    g_foc.data.duty_b = db;
    g_foc.data.duty_c = dc;

    /* Recalculate Valpha, Vbeta from ACTUAL (clamped) duty cycles.
     * The selective clamp may have modified ph[1] relative to the ideal SVPWM output.
     * SMO observer integrates Valpha/Vbeta to predict phase currents — it must use
     * the voltage actually applied to the motor, not the pre-clamp command. */
    g_foc.data.Valpha = (2.0f * da - db - dc) * ONE_THIRD * g_foc.data.Vbus;
    g_foc.data.Vbeta = (db - dc) * SQRT3_INV * g_foc.data.Vbus;
}
