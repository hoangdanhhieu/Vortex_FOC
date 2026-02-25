#ifndef CORDIC_MATH_H
#define CORDIC_MATH_H

#include <stdint.h>

#include "stm32g4xx_ll_cordic.h"

#define CORDIC_SCALE_FACTOR 2147483647.0f
#define CORDIC_Q31_TO_FLOAT 4.656612873e-10f

#define CORDIC_CFG_SINCOS                                                          \
    (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_4CYCLES | LL_CORDIC_SCALE_0 | \
     LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_2 | LL_CORDIC_INSIZE_32BITS |          \
     LL_CORDIC_OUTSIZE_32BITS)

#define CORDIC_CFG_ATAN2                                                          \
    (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_4CYCLES | LL_CORDIC_SCALE_0 | \
     LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_2 | LL_CORDIC_INSIZE_32BITS |         \
     LL_CORDIC_OUTSIZE_32BITS)

#define CORDIC_CFG_MODULUS                                                          \
    (LL_CORDIC_FUNCTION_MODULUS | LL_CORDIC_PRECISION_4CYCLES | LL_CORDIC_SCALE_0 | \
     LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_1 | LL_CORDIC_INSIZE_32BITS |           \
     LL_CORDIC_OUTSIZE_32BITS)
/**
 * @brief Calculate cosine and sine using CORDIC hardware
 * @param angle_norm Normalized angle [-1, 1) representing [-π, π)
 * @param cos_out Pointer to store cosine result
 * @param sin_out Pointer to store sine result
 */
static void inline cordic_sincos(float angle_norm, float* cos_out, float* sin_out) {
    WRITE_REG(CORDIC->CSR, CORDIC_CFG_SINCOS);
    __DSB();

    int32_t angle_q31 = (int32_t)(angle_norm * CORDIC_SCALE_FACTOR);

    int32_t modulus_q31 = 0x7FFFFFFF;

    LL_CORDIC_WriteData(CORDIC, (uint32_t)angle_q31);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)modulus_q31);

    int32_t cos_raw = (int32_t)LL_CORDIC_ReadData(CORDIC);
    int32_t sin_raw = (int32_t)LL_CORDIC_ReadData(CORDIC);

    *cos_out = (float)cos_raw * CORDIC_Q31_TO_FLOAT;
    *sin_out = (float)sin_raw * CORDIC_Q31_TO_FLOAT;
}

/**
 * @brief Calculate atan2(y, x) using CORDIC hardware
 * @return Normalized angle [-1, 1) representing [-π, π)
 */
static float inline cordic_atan2(float y, float x) {
    float abs_x = (x >= 0.0f) ? x : -x;
    float abs_y = (y >= 0.0f) ? y : -y;
    float max_val = (abs_x > abs_y) ? abs_x : abs_y;
    if (max_val < 1e-9f) return 0.0f;

    float scale = CORDIC_SCALE_FACTOR / max_val;
    int32_t x_q31 = (int32_t)(x * scale);
    int32_t y_q31 = (int32_t)(y * scale);

    WRITE_REG(CORDIC->CSR, CORDIC_CFG_ATAN2);
    __DSB();

    LL_CORDIC_WriteData(CORDIC, (uint32_t)x_q31);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)y_q31);

    int32_t phase_raw = (int32_t)LL_CORDIC_ReadData(CORDIC);
    (void)LL_CORDIC_ReadData(CORDIC);

    return (float)phase_raw * CORDIC_Q31_TO_FLOAT;
}

/**
 * @brief Calculate vector modulus sqrt(x² + y²) using CORDIC hardware
 * @param x First component
 * @param y Second component
 * @return  sqrt(x² + y²)
 *
 * @note Uses dedicated CORDIC MODULUS function.
 *       Inputs are normalized to max(|x|,|y|) to fit Q1.31,
 *       result is scaled back. Single CORDIC call, ~6 cycles.
 */
static inline float cordic_modulus(float x, float y) {
    float abs_x = (x >= 0.0f) ? x : -x;
    float abs_y = (y >= 0.0f) ? y : -y;
    float max_val = (abs_x > abs_y) ? abs_x : abs_y;
    if (max_val < 1e-9f) return 0.0f;

    float inv_max = CORDIC_SCALE_FACTOR / max_val;
    int32_t x_q31 = (int32_t)(x * inv_max);
    int32_t y_q31 = (int32_t)(y * inv_max);

    WRITE_REG(CORDIC->CSR, CORDIC_CFG_MODULUS);
    __DSB();

    LL_CORDIC_WriteData(CORDIC, (uint32_t)x_q31);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)y_q31);

    int32_t mod_raw = (int32_t)LL_CORDIC_ReadData(CORDIC);

    return (float)mod_raw * CORDIC_Q31_TO_FLOAT * max_val;
}

#endif /* CORDIC_MATH_H */