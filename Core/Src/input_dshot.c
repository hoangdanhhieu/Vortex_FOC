/**
 * @file input_dshot.c
 * @brief High-performance DShot receiver and telemetry driver implementation
 *
 * Implements DShot 150/300/600/1200 decoding, automatic baudrate detection,
 * self-aligning DMA capture, 3D throttle mode, and Bidirectional DShot (BDShot)
 * telemetry support on PB7 (TIM4 CH2).
 */

#include "input_dshot.h"

#include <string.h>

#include "flash_config.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "stm32g4xx_ll_dmamux.h"

/*===========================================================================*/
/* Pre-computed Fast Register Masks & Constants (170 MHz)                    */
/*===========================================================================*/
/* IC2F = 3 (8 samples at f_CK_INT = 170 MHz -> ~47 ns hardware glitch filter) */
#define DSHOT_TIM4_CCMR1_RX (TIM_CCMR1_CC2S_0 | (0x3U << TIM_CCMR1_IC2F_Pos))
#define DSHOT_TIM4_CCMR1_TX (0x6U << TIM_CCMR1_OC2M_Pos)
#define DSHOT_TIM4_CCER_RX (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP)
#define DSHOT_TIM4_CCER_TX (TIM_CCER_CC2E)
#define DSHOT_TIM4_DIER_RX (TIM_DIER_CC2DE)
#define DSHOT_TIM4_DIER_TX (TIM_DIER_UDE)

#define DSHOT_DMAMUX_REQ_RX (LL_DMAMUX_REQ_TIM4_CH2)
#define DSHOT_DMAMUX_REQ_TX (LL_DMAMUX_REQ_TIM4_UP)
#define DSHOT_DMA_CCR_RX                                                              \
    (DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_TCIE | \
     DMA_CCR_TEIE | DMA_CCR_EN)
#define DSHOT_DMA_CCR_TX (DSHOT_DMA_CCR_RX | DMA_CCR_DIR)
#define DSHOT_DMA_CLEAR_FLAGS (DMA_IFCR_CGIF2)
#define DSHOT_DMA_DISABLE_TIMEOUT 1000U

#define DSHOT_1D_INV_RANGE (1.0f / (float)(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN))
#define DSHOT_3D_INV_RANGE (1.0f / 999.0f)
#define DSHOT_3D_MIN_NORM 0.0001f

#define DSHOT_LOCK_FRAMES 5U
#define DSHOT_DETECT_ERR_RESET_THRESHOLD 4U
#define DSHOT_UNLOCK_ERR_THRESHOLD 8U
#define DSHOT_ARM_HOLD_MS 100U
#define DSHOT_ARM_MIN_FRAMES 10U

#define DSHOT_GCR_BUF_MAX_SIZE 68U
#define DSHOT_PAD_SLOTS_600 21U
#define DSHOT_PAD_SLOTS_300 11U
#define DSHOT_TX_FRAME_SLOTS 22U /* 1 start + 20 GCR bits + 1 trailing HIGH */

/* Compile-time timer tick profiles (+-20% detect, +-25% locked across 15-bit capture) */
#define DSHOT_SPEED_PROFILE(spd, baud_hz)                                           \
    {                                                                               \
        (spd),                                                                      \
        (uint16_t)(((uint64_t)SYSCLK_FREQ * 12ULL) / (uint64_t)(baud_hz)),          \
        (uint16_t)(((uint64_t)SYSCLK_FREQ * 18ULL) / (uint64_t)(baud_hz)),          \
        (uint16_t)(((uint64_t)SYSCLK_FREQ * 45ULL) / (4ULL * (uint64_t)(baud_hz))), \
        (uint16_t)(((uint64_t)SYSCLK_FREQ * 75ULL) / (4ULL * (uint64_t)(baud_hz))), \
    }

/*===========================================================================*/
/* Static Lookup Tables, Buffers & Driver Context                            */
/*===========================================================================*/
typedef struct {
    DShot_Speed_t speed;
    uint16_t dt15_detect_min;
    uint16_t dt15_detect_max;
    uint16_t dt15_min;
    uint16_t dt15_max;
} DShot_SpeedProfile_t;

typedef struct {
    /* Control outputs & failsafe timing */
    volatile float throttle_norm;
    volatile uint8_t arm_req;
    volatile uint8_t is_active;
    volatile uint8_t is_3d;
    volatile uint32_t last_frame_tick;

    /* Startup zero-throttle arming latch */
    volatile uint8_t armed;
    volatile uint16_t zero_frame_count;
    volatile uint32_t arm_zero_start_us;

    /* DShot Beacon command (1..5) debounce & latch */
    uint8_t beacon_last_cmd;
    uint8_t beacon_cmd_cnt;
    volatile uint8_t beacon_req;

    /* Speed & polarity lock state */
    volatile DShot_Speed_t speed;
    volatile DShot_State_t state;
    volatile uint8_t is_bdshot_locked;
    uint16_t threshold;
    uint16_t locked_dt15_min;
    uint16_t locked_dt15_max;
    uint8_t detect_count;
    uint8_t detect_is_bdshot;
    uint32_t detect_dt15_sum;
    uint8_t consec_errors;

    /* BDShot TX scheduling */
    volatile uint8_t bdshot_transmitting;
    volatile uint16_t tx_arr_ticks;
    uint8_t active_tx_buf;
    uint16_t precomputed_tx_len;

    /* Telemetry & diagnostics */
    volatile uint16_t raw_throttle;
    volatile uint8_t telemetry_req;
    volatile uint32_t frame_count;
    volatile uint32_t tx_frame_count;
    volatile uint32_t error_count;
    volatile uint32_t timeout_count;
} DShot_Ctx_t;

static const DShot_SpeedProfile_t k_speed_profiles[] = {
    DSHOT_SPEED_PROFILE(DSHOT_SPEED_1200, 1200000UL),
    DSHOT_SPEED_PROFILE(DSHOT_SPEED_600, 600000UL),
    DSHOT_SPEED_PROFILE(DSHOT_SPEED_300, 300000UL),
    DSHOT_SPEED_PROFILE(DSHOT_SPEED_150, 150000UL),
};

/* Standard 4B/5B GCR lookup table for nibbles 0x0 to 0xF */
static const uint8_t s_gcr_encode_table[16] = {
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
};

static volatile uint16_t s_dma_buf[DSHOT_DMA_BUF_SIZE] __attribute__((aligned(4)));
static uint16_t s_gcr_pwm_buf[2][DSHOT_GCR_BUF_MAX_SIZE] __attribute__((aligned(4)));
static DShot_Ctx_t s_dshot;
volatile uint32_t dshot_process_time;

/*===========================================================================*/
/* Low-Level Hardware & Critical Section Helpers                             */
/*===========================================================================*/

static inline uint32_t DShot_DisableIrq(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void DShot_RestoreIrq(uint32_t primask) {
    if (!primask) {
        __enable_irq();
    }
}

static inline uint8_t DShot_IsBdshotSpeed(DShot_Speed_t spd) {
    return (spd == DSHOT_SPEED_300 || spd == DSHOT_SPEED_600) ? 1U : 0U;
}

static inline uint16_t DShot_GetPadSlots(DShot_Speed_t spd) {
    return (spd == DSHOT_SPEED_600) ? DSHOT_PAD_SLOTS_600 : DSHOT_PAD_SLOTS_300;
}

/**
 * @brief Disable DMA1 CH2 and wait until active AHB transfers clear (RM0440 requirement)
 */
static inline void DShot_WaitDmaDisabled(void) {
    DMA1_Channel2->CCR = 0U;
    for (uint32_t wait = 0U;
         (DMA1_Channel2->CCR & DMA_CCR_EN) && (wait < DSHOT_DMA_DISABLE_TIMEOUT); wait++) {
    }
}

/**
 * @brief Reset TIM4 peripheral and wait for DMA1 CH2 to reach quiescent disabled state
 */
static inline void DShot_ResetTim4AndDma(uint32_t dmamux_req) {
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM4RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM4RST;

    DShot_WaitDmaDisabled();
    DMA1->IFCR = DSHOT_DMA_CLEAR_FLAGS;
    DMAMUX1_Channel1->CCR = dmamux_req;
}

/**
 * @brief Switch TIM4 CH2 and DMA1 CH2 back to input capture mode for throttle reception
 */
static inline void DShot_SwitchToReceive(void) {
    DShot_ResetTim4AndDma(DSHOT_DMAMUX_REQ_RX);

    DMA1_Channel2->CPAR = (uint32_t)&TIM4->CCR2;
    DMA1_Channel2->CMAR = (uint32_t)s_dma_buf;
    DMA1_Channel2->CNDTR = DSHOT_DMA_BUF_SIZE;

    TIM4->PSC = 0U;
    TIM4->ARR = 0xFFFFU;
    TIM4->CNT = 0U;
    TIM4->CCMR1 = DSHOT_TIM4_CCMR1_RX;
    TIM4->SR = 0U;

    s_dshot.bdshot_transmitting = 0U;
    DMA1_Channel2->CCR = DSHOT_DMA_CCR_RX;
    TIM4->DIER = DSHOT_TIM4_DIER_RX;
    TIM4->CCER = DSHOT_TIM4_CCER_RX;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Switch TIM4 CH2 and DMA1 CH2 to transmit 21-bit GCR telemetry frame
 * @param arr_ticks Timer auto-reload value corresponding to 1 bit slot duration
 * @param tx_buf Pointer to active telemetry PWM buffer
 * @param total_len Total DMA elements to transmit (padding + start + 20 GCR + trailing)
 */
static inline void DShot_SwitchToTransmit(uint16_t arr_ticks, const uint16_t* tx_buf,
                                          uint16_t total_len) {
    DShot_ResetTim4AndDma(DSHOT_DMAMUX_REQ_TX);

    TIM4->PSC = 0U;
    TIM4->ARR = arr_ticks;
    TIM4->CNT = 0U;
    TIM4->CCMR1 = DSHOT_TIM4_CCMR1_TX;
    TIM4->CCR2 = arr_ticks + 1U;
    TIM4->SR = 0U;

    DMA1_Channel2->CPAR = (uint32_t)&TIM4->CCR2;
    DMA1_Channel2->CMAR = (uint32_t)tx_buf;
    DMA1_Channel2->CNDTR = total_len;

    s_dshot.bdshot_transmitting = 1U;
    DMA1_Channel2->CCR = DSHOT_DMA_CCR_TX;
    TIM4->DIER = DSHOT_TIM4_DIER_TX;
    TIM4->CCER = DSHOT_TIM4_CCER_TX;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/*===========================================================================*/
/* Detection State & Error Helpers                                           */
/*===========================================================================*/

/**
 * @brief Reset baudrate/polarity lock state so the next valid sequence re-acquires cleanly
 */
static void DShot_ResetDetectionState(void) {
    s_dshot.state = DSHOT_STATE_DETECTING;
    s_dshot.is_bdshot_locked = 0U;
    s_dshot.detect_count = 0U;
    s_dshot.detect_is_bdshot = 0U;
    s_dshot.detect_dt15_sum = 0U;
    s_dshot.speed = DSHOT_SPEED_UNKNOWN;
    s_dshot.threshold = 0U;
    s_dshot.tx_arr_ticks = 0U;
    s_dshot.locked_dt15_min = 0U;
    s_dshot.locked_dt15_max = 0U;
    s_dshot.precomputed_tx_len = 0U;
    s_dshot.active_tx_buf = 0U;
    s_dshot.consec_errors = 0U;
}

/**
 * @brief Reset control outputs, startup arming latch, and detection lock state
 */
static void DShot_ResetControlAndDetection(void) {
    s_dshot.is_active = 0U;
    s_dshot.arm_req = 0U;
    s_dshot.throttle_norm = 0.0f;
    s_dshot.armed = 0U;
    s_dshot.zero_frame_count = 0U;
    s_dshot.arm_zero_start_us = 0U;
    s_dshot.beacon_last_cmd = 0U;
    s_dshot.beacon_cmd_cnt = 0U;
    s_dshot.beacon_req = 0U;
    s_dshot.bdshot_transmitting = 0U;
    DShot_ResetDetectionState();
}

/**
 * @brief Record a frame error and apply bounded detection/unlock reset thresholds
 */
static void DShot_HandleFrameError(void) {
    s_dshot.error_count++;

    if (s_dshot.consec_errors < UINT8_MAX) {
        s_dshot.consec_errors++;
    }

    if (s_dshot.state == DSHOT_STATE_DETECTING) {
        if (s_dshot.consec_errors >= DSHOT_DETECT_ERR_RESET_THRESHOLD) {
            DShot_ResetDetectionState();
        }
    } else if (!s_dshot.armed && (s_dshot.consec_errors >= DSHOT_UNLOCK_ERR_THRESHOLD)) {
        DShot_ResetDetectionState();
    }
}

/*===========================================================================*/
/* BDShot GCR Telemetry Encoding                                             */
/*===========================================================================*/

/**
 * @brief Pre-fill leading HIGH padding slots in both ping-pong telemetry buffers
 */
static void DShot_InitPaddingBuffers(uint16_t arr_ticks, uint16_t n_pad) {
    uint16_t high_val = arr_ticks + 1U;
    for (uint16_t b = 0U; b < 2U; b++) {
        for (uint16_t i = 0U; i < n_pad; i++) {
            s_gcr_pwm_buf[b][i] = high_val;
        }
    }
}

/**
 * @brief Pack current motor eRPM into a 21-bit GCR telemetry frame with DMA HIGH padding
 * @param arr_ticks TX Timer auto-reload value for 1 bit period
 * @param out_buf Pointer to destination buffer for PWM pulse widths
 * @return Total number of halfwords to transmit via DMA
 */
static uint16_t DShot_PrepareTelemetryFrame(uint16_t arr_ticks, uint16_t* out_buf) {
    uint16_t data12;
    float omega_elec = FOC_GetSpeed();
    float abs_speed = (omega_elec >= 0.0f) ? omega_elec : -omega_elec;
    FOC_State_t foc_state = FOC_GetStatus()->state;

    uint8_t motor_stopped = (foc_state == FOC_STATE_IDLE || foc_state == FOC_STATE_STOP ||
                             foc_state == FOC_STATE_FAULT || foc_state == FOC_STATE_BEEP ||
                             foc_state == FOC_STATE_CALIBRATION || foc_state == FOC_STATE_ALIGN ||
                             foc_state == FOC_STATE_DETECT || abs_speed < 10.0f);

    if (motor_stopped) {
        data12 = 0x0FFFU; /* Standard Betaflight motor stopped convention */
    } else {
        /* Commutation period: period_us = (2 * PI * 1000000) / |omega_elec| */
        uint32_t period_us = (uint32_t)(6283185.3f / abs_speed);
        if (period_us <= 0x1FFU) {
            data12 = (uint16_t)period_us;
        } else {
            /* Hardware-accelerated mantissa exponent compression via __builtin_clz (1 cycle) */
            uint32_t clz = (uint32_t)__builtin_clz(period_us);
            uint32_t shift = (clz < 23U) ? (23U - clz) : 0U;
            uint8_t e = (shift > 7U) ? 7U : (uint8_t)shift;
            uint16_t m = (uint16_t)(period_us >> e);
            if (m > 0x1FFU) {
                m = 0x1FFU;
            }
            data12 = (uint16_t)((e << 9) | (m & 0x1FFU));
        }
    }

    /* 4-bit Inverted Checksum */
    uint8_t crc = (uint8_t)((~(data12 ^ (data12 >> 4) ^ (data12 >> 8))) & 0x0FU);
    uint16_t packet16 = (uint16_t)((data12 << 4) | crc);

    /* GCR 4B/5B Encoding (20 bits) */
    uint32_t gcr20 = ((uint32_t)s_gcr_encode_table[(packet16 >> 12) & 0x0FU] << 15) |
                     ((uint32_t)s_gcr_encode_table[(packet16 >> 8) & 0x0FU] << 10) |
                     ((uint32_t)s_gcr_encode_table[(packet16 >> 4) & 0x0FU] << 5) |
                     ((uint32_t)s_gcr_encode_table[packet16 & 0x0FU]);

    uint16_t high_val = arr_ticks + 1U;
    uint16_t n_pad = DShot_GetPadSlots(s_dshot.speed);

    /* 1. Start slot: 0% duty (LOW) creates initial falling edge (slots 0..n_pad-1 pre-filled) */
    out_buf[n_pad] = 0U;

    /* 2. NRZI wire modulation for 20 GCR bits */
    uint8_t wire_state = 0U;
    for (uint32_t i = 0U; i < 20U; i++) {
        uint8_t bit = (uint8_t)((gcr20 >> (19U - i)) & 1U);
        wire_state ^= bit; /* '1' toggles state, '0' maintains state */
        out_buf[n_pad + 1U + i] = wire_state ? high_val : 0U;
    }

    /* 3. Trailing idle slot: keep wire at HIGH for seamless handoff back to RX */
    out_buf[n_pad + 21U] = high_val;

    return (uint16_t)(n_pad + DSHOT_TX_FRAME_SLOTS);
}

/*===========================================================================*/
/* Protocol Processing & Decoding                                            */
/*===========================================================================*/

/**
 * @brief Match DShot speed profile from 15-bit capture span
 * @param dt15 Measured 15-bit capture span in timer ticks (buf[30] - buf[0])
 * @return Pointer to matching static speed profile, or NULL if out of range
 */
static const DShot_SpeedProfile_t* DShot_AutoDetectSpeed(uint16_t dt15) {
    for (uint32_t i = 0U; i < (sizeof(k_speed_profiles) / sizeof(k_speed_profiles[0])); i++) {
        if (dt15 >= k_speed_profiles[i].dt15_detect_min &&
            dt15 <= k_speed_profiles[i].dt15_detect_max) {
            return &k_speed_profiles[i];
        }
    }
    return NULL;
}

/**
 * @brief Pure frame decoder: computes per-frame bit threshold, extracts 16 bits, verifies CRC
 * @param buf DMA capture buffer (32 timestamps for 16 bits)
 * @param dt15 Measured 15-bit capture span (buf[30] - buf[0])
 * @param out_data Decoded 12-bit payload (11-bit throttle + 1-bit telemetry request)
 * @param out_is_bdshot Set to 1 if inverted BDShot CRC matched, 0 for Standard DShot
 * @return 1 if CRC matched, 0 on invalid frame
 */
static uint8_t DShot_DecodeFrame(const volatile uint16_t* buf, uint16_t dt15, uint16_t* out_data,
                                 uint8_t* out_is_bdshot) {
    uint16_t raw_frame = 0U;
    /* threshold = 9/16 * T_bit = (9/16) * (dt15 / 15) = (dt15 * 3) / 80 with half-up rounding */
    const uint16_t threshold = (uint16_t)(((uint32_t)dt15 * 3U + 40U) / 80U);

    for (uint32_t i = 0U; i < 16U; i++) {
        uint16_t pulse_len = (uint16_t)(buf[2U * i + 1U] - buf[2U * i]);
        raw_frame = (uint16_t)((raw_frame << 1) | ((pulse_len > threshold) ? 1U : 0U));
    }

    uint16_t data = raw_frame >> 4;
    uint8_t rx_crc = (uint8_t)(raw_frame & 0x0FU);
    uint8_t calc_crc = (uint8_t)((data ^ (data >> 4) ^ (data >> 8)) & 0x0FU);
    uint8_t inv_crc = (uint8_t)((~calc_crc) & 0x0FU);

    if (s_dshot.state == DSHOT_STATE_LOCKED) {
        uint8_t expected = s_dshot.is_bdshot_locked ? inv_crc : calc_crc;
        if (rx_crc != expected) {
            return 0U;
        }
        *out_is_bdshot = s_dshot.is_bdshot_locked;
    } else if (rx_crc == calc_crc) {
        *out_is_bdshot = 0U;
    } else if (DShot_IsBdshotSpeed(s_dshot.speed) && (rx_crc == inv_crc)) {
        *out_is_bdshot = 1U;
    } else {
        return 0U;
    }

    s_dshot.threshold = threshold;
    *out_data = data;
    return 1U;
}

/**
 * @brief Advance detection counter uniformly for Standard or BDShot and lock TX timing at 5 frames
 */
static void DShot_UpdateLockState(uint16_t dt15, uint8_t is_bdshot) {
    if (s_dshot.state == DSHOT_STATE_LOCKED) {
        return;
    }

    if ((s_dshot.detect_count > 0U) && (s_dshot.detect_is_bdshot != is_bdshot)) {
        s_dshot.detect_count = 0U;
        s_dshot.detect_dt15_sum = 0U;
    }

    s_dshot.detect_is_bdshot = is_bdshot;
    s_dshot.detect_dt15_sum += dt15;

    if (++s_dshot.detect_count >= DSHOT_LOCK_FRAMES) {
        s_dshot.is_bdshot_locked = is_bdshot;
        if (is_bdshot) {
            /* Average T_tx_bit = (4 / 5) * mean(T_rx_bit) = (dt15_sum * 4) / (75 * count) */
            uint32_t denom = 75U * (uint32_t)s_dshot.detect_count;
            uint32_t tx_period = (s_dshot.detect_dt15_sum * 4U + (denom >> 1)) / denom;
            s_dshot.tx_arr_ticks = (tx_period > 1U) ? (uint16_t)(tx_period - 1U) : 0U;
            DShot_InitPaddingBuffers(s_dshot.tx_arr_ticks, DShot_GetPadSlots(s_dshot.speed));
        } else {
            s_dshot.tx_arr_ticks = 0U;
        }
        s_dshot.state = DSHOT_STATE_LOCKED;
    }
}

/**
 * @brief Map 11-bit raw throttle value to normalized throttle, arming, and direction
 */
static void DShot_MapThrottle(uint16_t raw, uint8_t is_3d, uint32_t now_us, float* out_thr,
                              uint8_t* out_arm) {
    if (!s_dshot.armed) {
        if (raw == 0U) {
            if (s_dshot.zero_frame_count == 0U) {
                s_dshot.arm_zero_start_us = now_us;
            }
            if (s_dshot.zero_frame_count < UINT16_MAX) {
                s_dshot.zero_frame_count++;
            }
            if ((s_dshot.state == DSHOT_STATE_LOCKED) &&
                (s_dshot.zero_frame_count >= DSHOT_ARM_MIN_FRAMES) &&
                ((now_us - s_dshot.arm_zero_start_us) >= (DSHOT_ARM_HOLD_MS * 1000U))) {
                s_dshot.armed = 1U;
            }
        } else {
            s_dshot.zero_frame_count = 0U;
        }
        *out_thr = 0.0f;
        *out_arm = 0U;
        return;
    }

    if (raw < DSHOT_THROTTLE_MIN) {
        *out_thr = 0.0f;
        *out_arm = 0U;
        return;
    }

    float norm = 0.0f;
    if (is_3d) {
        /* Standard Betaflight 3D split: 48..1047 = reverse, 1048..2047 = forward */
        if (raw < DSHOT_3D_DEADBAND) {
            float mag = (float)(raw - DSHOT_THROTTLE_MIN) * DSHOT_3D_INV_RANGE;
            norm = -clampf(mag, DSHOT_3D_MIN_NORM, 1.0f);
        } else {
            float mag = (float)(raw - DSHOT_3D_DEADBAND) * DSHOT_3D_INV_RANGE;
            norm = clampf(mag, DSHOT_3D_MIN_NORM, 1.0f);
        }
    } else {
        norm = (float)(raw - DSHOT_THROTTLE_MIN) * DSHOT_1D_INV_RANGE;
        norm = clampf(norm, 0.0f, 1.0f);
    }

    *out_thr = norm;
    *out_arm = 1U;
}

/**
 * @brief Process newly captured DMA frame in interrupt context
 * @return 1 if frame passed timing + CRC validation, 0 on error
 */
static uint8_t DShot_ProcessReceivedFrame(void) {
    uint16_t dt15 = (uint16_t)(s_dma_buf[30] - s_dma_buf[0]);

    if (s_dshot.state == DSHOT_STATE_DETECTING) {
        const DShot_SpeedProfile_t* profile = DShot_AutoDetectSpeed(dt15);
        if (profile == NULL) {
            DShot_HandleFrameError();
            return 0U;
        }
        if (profile->speed != s_dshot.speed) {
            if (s_dshot.speed != DSHOT_SPEED_UNKNOWN) {
                s_dshot.detect_count = 0U;
                s_dshot.detect_dt15_sum = 0U;
            }
            s_dshot.speed = profile->speed;
            s_dshot.locked_dt15_min = profile->dt15_min;
            s_dshot.locked_dt15_max = profile->dt15_max;
        }
    } else if ((dt15 < s_dshot.locked_dt15_min) || (dt15 > s_dshot.locked_dt15_max)) {
        DShot_HandleFrameError();
        return 0U;
    }

    uint16_t data = 0U;
    uint8_t is_bdshot = 0U;
    if (!DShot_DecodeFrame(s_dma_buf, dt15, &data, &is_bdshot)) {
        DShot_HandleFrameError();
        return 0U;
    }

    uint32_t now_us = FOC_Get_1MhzCounter();
    s_dshot.last_frame_tick = now_us;
    s_dshot.frame_count++;
    s_dshot.consec_errors = 0U;

    s_dshot.raw_throttle = data >> 1;
    s_dshot.telemetry_req = (uint8_t)(data & 0x01U);

    DShot_UpdateLockState(dt15, is_bdshot);

    if (s_dshot.state == DSHOT_STATE_LOCKED) {
        s_dshot.is_active = 1U;
        float thr = 0.0f;
        uint8_t arm = 0U;
        DShot_MapThrottle(s_dshot.raw_throttle, s_dshot.is_3d, now_us, &thr, &arm);
        s_dshot.throttle_norm = thr;
        s_dshot.arm_req = arm;
        FOC_Input_PushThrottle(thr);
        /* Decode DShot Beacon commands (1..5) while disarmed/idle */
        if ((s_dshot.raw_throttle >= 1U) && (s_dshot.raw_throttle <= 5U)) {
            uint8_t bcmd = (uint8_t)s_dshot.raw_throttle;
            if (bcmd == s_dshot.beacon_last_cmd) {
                if (s_dshot.beacon_cmd_cnt < 3U) {
                    s_dshot.beacon_cmd_cnt++;
                }
            } else {
                s_dshot.beacon_last_cmd = bcmd;
                s_dshot.beacon_cmd_cnt = 1U;
            }
            if (s_dshot.beacon_cmd_cnt >= 3U) {
                s_dshot.beacon_req = bcmd;
            }
        } else {
            s_dshot.beacon_last_cmd = 0U;
            s_dshot.beacon_cmd_cnt = 0U;
        }
    }

    return 1U;
}

/*===========================================================================*/
/* Interrupt Handlers & Driver Interfaces                                    */
/*===========================================================================*/

/**
 * @brief Main DMA completion callback executing in interrupt context
 */
void DShot_DMA_IRQHandler(void) {
    if (s_dshot.bdshot_transmitting) {
        s_dshot.tx_frame_count++;
        DShot_SwitchToReceive();
        return;
    }

    DShot_WaitDmaDisabled();

    /* 1. Validate RX frame BEFORE starting TX telemetry response */
    uint32_t start_time = FOC_Get_1MhzCounter();
    uint8_t frame_ok = DShot_ProcessReceivedFrame();
    dshot_process_time = FOC_Get_1MhzCounter() - start_time;

    /* 2. For locked BDShot, transmit precomputed telemetry and queue next buffer */
    if (frame_ok && s_dshot.is_bdshot_locked && DShot_IsBdshotSpeed(s_dshot.speed)) {
        if (s_dshot.precomputed_tx_len > 0U) {
            DShot_SwitchToTransmit(s_dshot.tx_arr_ticks, s_gcr_pwm_buf[s_dshot.active_tx_buf],
                                   s_dshot.precomputed_tx_len);
        }

        uint8_t next_buf_idx = 1U - s_dshot.active_tx_buf;
        s_dshot.precomputed_tx_len =
            DShot_PrepareTelemetryFrame(s_dshot.tx_arr_ticks, s_gcr_pwm_buf[next_buf_idx]);
        s_dshot.active_tx_buf = next_buf_idx;
    }

    /* 3. Immediately re-arm receiver if TX was not launched */
    if (!s_dshot.bdshot_transmitting) {
        DShot_SwitchToReceive();
    }
}

/**
 * @brief Recovery hook executed in interrupt context after a DMA transfer error (TEIF2)
 *
 * Counts the error and re-arms RX capture via full TIM4/DMA reset.
 */
void DShot_DMA_ErrorRecover(void) {
    s_dshot.error_count++;
    DShot_SwitchToReceive();
}

/**
 * @brief Initialize DShot hardware peripherals and start DMA reception
 */
static void DShot_Init(void) {
    uint32_t primask = DShot_DisableIrq();
    FlashConfig_t* cfg = FlashConfig_Get();

    memset(&s_dshot, 0, sizeof(s_dshot));
    s_dshot.is_3d = (cfg != NULL) ? cfg->dshot_3d_enable : 0U;
    DShot_ResetDetectionState();
    s_dshot.last_frame_tick = FOC_Get_1MhzCounter();

    DShot_SwitchToReceive();
    DShot_RestoreIrq(primask);
}

/**
 * @brief Stop TIM4/DMA1 CH2 and clear DShot state when switching input drivers
 */
static void DShot_Deinit(void) {
    uint32_t primask = DShot_DisableIrq();

    TIM4->DIER = 0U;
    TIM4->CCER = 0U;
    TIM4->CR1 = 0U;
    DShot_WaitDmaDisabled();
    TIM4->SR = 0U;
    DMA1->IFCR = DSHOT_DMA_CLEAR_FLAGS;

    DShot_ResetControlAndDetection();

    DShot_RestoreIrq(primask);
}

/**
 * @brief 1 kHz background polling hook for failsafe and arming management
 */
static void DShot_Update(FOC_InputCmd_t* cmd) {
    if (cmd == NULL) {
        return;
    }

    FlashConfig_t* cfg = FlashConfig_Get();
    uint8_t is_3d = (cfg != NULL) ? cfg->dshot_3d_enable : 0U;

    uint32_t primask = DShot_DisableIrq();
    s_dshot.is_3d = is_3d;

    uint32_t now_us = FOC_Get_1MhzCounter();
    if (s_dshot.is_active &&
        ((now_us - s_dshot.last_frame_tick) > (DSHOT_SIGNAL_TIMEOUT_MS * 1000U))) {
        s_dshot.timeout_count++;
        DShot_ResetControlAndDetection();
        DShot_SwitchToReceive();
    }

    cmd->is_active = s_dshot.is_active;
    cmd->arm_req = s_dshot.arm_req;
    cmd->is_armed_ready = s_dshot.armed;
    cmd->beacon_req = s_dshot.beacon_req;
    s_dshot.beacon_req = 0U;
    DShot_RestoreIrq(primask);
}

/**
 * @brief Fast normalized throttle query for 48 kHz HF execution
 */
CCMRAM_FUNC float DShot_ReadThr(void) {
    return s_dshot.throttle_norm;
}

/**
 * @brief Get real-time DShot telemetry and diagnostic statistics
 */
void DShot_GetStats(DShot_Stats_t* stats) {
    if (stats == NULL) {
        return;
    }

    uint32_t primask = DShot_DisableIrq();
    stats->speed = s_dshot.speed;
    stats->state = s_dshot.state;
    stats->is_bdshot_locked = s_dshot.is_bdshot_locked;
    stats->is_armed = s_dshot.armed;
    stats->telemetry_req = s_dshot.telemetry_req;
    stats->raw_throttle = s_dshot.raw_throttle;
    stats->frame_count = s_dshot.frame_count;
    stats->tx_frame_count = s_dshot.tx_frame_count;
    stats->error_count = s_dshot.error_count;
    stats->timeout_count = s_dshot.timeout_count;
    DShot_RestoreIrq(primask);
}

/*===========================================================================*/
/* Pluggable FOC Input Driver Export                                         */
/*===========================================================================*/

const FOC_InputDriver_t g_driver_dshot = {
    .name = "DShot",
    .init = DShot_Init,
    .deinit = DShot_Deinit,
    .update = DShot_Update,
    .read_throttle = DShot_ReadThr,
};
