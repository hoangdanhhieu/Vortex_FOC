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
    float va_norm = g_foc.data.Valpha * Vbus_inv;
    float vb_norm = g_foc.data.Vbeta * Vbus_inv;

    /* Clamp voltage vector magnitude to SVPWM linear range considering max_duty.
     * To prevent clipping/flattening the sine peaks when max_duty < 1.0,
     * we scale the maximum linear voltage magnitude proportionally:
     * V_max_limit = (1 / sqrt(3)) * 2 * (max_duty - 0.5) */
    float max_duty = g_foc.max_duty;
    float v_max_limit = SQRT3_INV * 2.0f * (max_duty - 0.5f);
    if (v_max_limit < 0.0f) v_max_limit = 0.0f;
    float v_max_limit_sq = v_max_limit * v_max_limit;
    float v_sq = va_norm * va_norm + vb_norm * vb_norm;
    if (v_sq > v_max_limit_sq) {
        float v_mag = cordic_modulus(va_norm, vb_norm);
        if (v_mag > 1e-6f) {
            float scale = v_max_limit / v_mag;
            va_norm *= scale;
            vb_norm *= scale;
        }
    }

    g_foc.data.Valpha = va_norm * g_foc.data.Vbus;
    g_foc.data.Vbeta = vb_norm * g_foc.data.Vbus;

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
