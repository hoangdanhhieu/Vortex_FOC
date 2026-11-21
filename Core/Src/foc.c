#include "foc.h"

#include "cordic_math.h"
#include "foc_config.h"
#include "foc_state_machine.h"

/*===========================================================================*/
/* Space Vector PWM                                                          */
/*===========================================================================*/

NOINLINE_FUNC CCMRAM_FUNC void svpwm_calculate(float theta) {
    float Vbus_inv = g_foc.data.Vbus_inv;

    float v_max_limit = g_foc.v_max_scale_z1;
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

    float cos_th, sin_th;
    cordic_sincos(theta, &cos_th, &sin_th);

    float va_norm, vb_norm;
    foc_svpwm_dq(vd_norm, vq_norm, cos_th, sin_th, g_foc.max_duty, 0U, &va_norm, &vb_norm,
                 &g_foc.data.duty_a, &g_foc.data.duty_b, &g_foc.data.duty_c);

    g_foc.data.Valpha = va_norm * g_foc.data.Vbus;
    g_foc.data.Vbeta = vb_norm * g_foc.data.Vbus;
}
