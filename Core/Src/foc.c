#include "foc.h"

#include <main.h>

#include "cordic_math.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "stm32g4xx_ll_tim.h"

/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

/*
 * Dead-time compensation constant (Duty cycle equivalent)
 * D_dead = T_dead / T_pwm = T_dead * F_pwm
 * Factor 2.0 covers total loss (rise+fall or effective pulse width reduction)
 */
#define DEAD_TIME_DUTY (DEAD_TIME_NS * 1e-9f * (float)PWM_FREQUENCY * 2.0f)
CCMRAM_FUNC void svpwm_calculate(void) {
    /* Safety: Vbus_inv already precomputed in FOC_HighFrequencyTask */
    float Vbus_inv = g_foc.data.Vbus_inv;

    /* Normalize voltages to Vbus */
    float va_norm = g_foc.data.Valpha * Vbus_inv;
    float vb_norm = g_foc.data.Vbeta * Vbus_inv;

    /* Clamp voltage vector magnitude to SVPWM linear range */
    float v_sq = va_norm * va_norm + vb_norm * vb_norm;
    const float V_MAX_SQ = ONE_THIRD; /* (1/sqrt(3))² = 1/3 */
    if (v_sq > V_MAX_SQ) {
        float v_mag = cordic_modulus(va_norm, vb_norm);
        float scale = SQRT3_INV / v_mag;
        va_norm *= scale;
        vb_norm *= scale;
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

    /* Apply Dead-time Compensation directly to Duty Cycles */
    const float I_threshold = 0.5f;
    float dt_comp = DEAD_TIME_DUTY;

    if (g_foc.data.Ia > I_threshold)
        da += dt_comp;
    else if (g_foc.data.Ia < -I_threshold)
        da -= dt_comp;

    if (g_foc.data.Ib > I_threshold)
        db += dt_comp;
    else if (g_foc.data.Ib < -I_threshold)
        db -= dt_comp;

    if (g_foc.data.Ic > I_threshold)
        dc += dt_comp;
    else if (g_foc.data.Ic < -I_threshold)
        dc -= dt_comp;

    /* Clamp duty cycles to valid range [0, 1] */
    float max_duty = g_foc.max_duty;
    g_foc.data.duty_a = da < 0.0f ? 0.0f : (da > max_duty ? max_duty : da);
    g_foc.data.duty_b = db < 0.0f ? 0.0f : (db > max_duty ? max_duty : db);
    g_foc.data.duty_c = dc < 0.0f ? 0.0f : (dc > max_duty ? max_duty : dc);
}
