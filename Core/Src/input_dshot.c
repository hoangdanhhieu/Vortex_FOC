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
#include "stm32g4xx_ll_gpio.h"

/*===========================================================================*/
/* Pre-computed Fast Register Masks for TIM4 & DMA1_CH2 (170 MHz)            */
/*===========================================================================*/

/* --- 1. TIM4 CCMR1 (Channel 2) --- */
/* RX: CC2S = 01 (Input Capture mapped on TI2), IC2F = 0000 (No digital filter) */
#define DSHOT_TIM4_CCMR1_RX (TIM_CCMR1_CC2S_0)

/* TX: CC2S = 00 (Output), OC2M = 110 (PWM Mode 1), OC2PE = 0 (Preload disabled) */
#define DSHOT_TIM4_CCMR1_TX (0x6U << TIM_CCMR1_OC2M_Pos)

/* --- 2. TIM4 CCER (Channel 2 Enable & Polarity) --- */
/* RX: CC2E = 1, CC2P = 1, CC2NP = 1 -> Capture both rising and falling edges */
#define DSHOT_TIM4_CCER_RX (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP)

/* TX: CC2E = 1, CC2P = 0, CC2NP = 0 -> Output compare active high */
#define DSHOT_TIM4_CCER_TX (TIM_CCER_CC2E)

/* --- 3. TIM4 DIER (DMA Request Trigger) --- */
/* RX: CC2DE = 1 -> Trigger DMA on Capture Compare 2 edge event */
#define DSHOT_TIM4_DIER_RX (TIM_DIER_CC2DE)

/* TX: UDE = 1 -> Trigger DMA on Timer Update (overflow) event per bit */
#define DSHOT_TIM4_DIER_TX (TIM_DIER_UDE)

/* --- 4. DMAMUX1 Channel 1 (Maps to DMA1_CH2) --- */
#define DSHOT_DMAMUX_REQ_RX (LL_DMAMUX_REQ_TIM4_CH2) /* 0x44U = 68 */
#define DSHOT_DMAMUX_REQ_TX (LL_DMAMUX_REQ_TIM4_UP)  /* 0x47U = 71 */

/* --- 5. DMA1 Channel 2 CCR --- */
/* Low Priority, MSIZE=16, PSIZE=16, Memory Increment, TCIE, TEIE, Channel Enable */
/* RX: DIR = 0 (Peripheral-to-Memory) -> 0x0000158B */
#define DSHOT_DMA_CCR_RX                                                              \
    (DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_TCIE | \
     DMA_CCR_TEIE | DMA_CCR_EN)

/* TX: DIR = 1 (Memory-to-Peripheral) -> 0x0000159B */
#define DSHOT_DMA_CCR_TX (DSHOT_DMA_CCR_RX | DMA_CCR_DIR)

/* --- 6. Clear DMA1 Channel 2 Interrupt Flags --- */
#define DSHOT_DMA_CLEAR_FLAGS (DMA_IFCR_CGIF2)

/* Pre-computed reciprocal scale factors for fast throttle normalization without FPU division */
#define DSHOT_1D_INV_RANGE \
    (1.0f / (float)(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)) /* 1.0f / 1999.0f */
#define DSHOT_3D_REV_INV_RANGE (1.0f / 999.0f)
#define DSHOT_3D_FWD_INV_RANGE (1.0f / 998.0f)

/*===========================================================================*/
/* Static Driver Buffers & State Variables                                   */
/*===========================================================================*/

/* Raw DMA capture buffer for 16 bits * 2 edge timestamps = 32 halfwords */
static uint16_t s_dma_buf[DSHOT_DMA_BUF_SIZE] __attribute__((aligned(4)));

/* Output buffer for BDShot telemetry: 2 ping-pong buffers of up to 68 halfwords each */
#define DSHOT_GCR_BUF_MAX_SIZE 68
static uint16_t s_gcr_pwm_buf[2][DSHOT_GCR_BUF_MAX_SIZE] __attribute__((aligned(4)));
static uint8_t s_active_tx_buf = 0;
static uint16_t s_precomputed_tx_len = 0;

/* Normalized fast output variables */
static volatile float s_throttle_norm = 0.0f;
static volatile uint8_t s_arm_req = 0;
static volatile int8_t s_dir_req = 0;
static volatile uint8_t s_is_active = 0;
static volatile uint8_t s_bdshot_transmitting = 0;

/* Safety handshake interlock state */
static uint8_t s_dshot_armed = 0;
static uint16_t s_zero_frame_count = 0;

/* Protocol status and auto-detection */
static DShot_Speed_t s_speed = DSHOT_SPEED_UNKNOWN;
static DShot_State_t s_state = DSHOT_STATE_DETECTING;
static uint16_t s_threshold = 0;
static uint8_t s_is_bdshot_locked = 0;
static uint8_t s_standard_detect_count = 0;
static uint8_t s_bdshot_detect_count = 0;
static uint32_t s_last_frame_tick = 0;
static uint16_t s_raw_throttle = 0;
static uint8_t s_telemetry_req = 0;
static uint8_t s_dshot_is_3d = 0;

/* Diagnostic statistics */
static uint32_t s_frame_count = 0;
static uint32_t s_tx_frame_count = 0;
static uint32_t s_error_count = 0;
static uint32_t s_timeout_count = 0;
static uint16_t s_tx_arr_ticks = 0;
static float rx_time_per_tick;
volatile uint32_t dshot_process_time;
/* Forward declarations */
static void DShot_Init(void);
static void DShot_Read(FOC_InputCmd_t* cmd);
static void DShot_ProcessReceivedFrame(void);

/*===========================================================================*/
/* Fast Hardware Switch Functions                                            */
/*===========================================================================*/

/**
 * @brief Switch TIM4 CH2 and DMA1 CH2 to transmit 21-bit GCR telemetry frame
 * @param arr_ticks Timer auto-reload value corresponding to 1 bit slot duration
 * @param tx_buf Pointer to active telemetry PWM buffer
 * @param total_len Total DMA elements to transmit (padding + start + 20 GCR + trailing)
 */
static inline void DShot_SwitchToTransmit(uint16_t arr_ticks, const uint16_t* tx_buf,
                                          uint16_t total_len) {
    /* 1. Hard reset TIM4 via RCC to clean all internal counters, flags, and registers */
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM4RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM4RST;

    /* 2. Reset DMA channel and clear flags */
    DMA1_Channel2->CCR = 0;
    DMA1->IFCR = DSHOT_DMA_CLEAR_FLAGS;

    /* 3. Route DMAMUX to TIM4 update event trigger */
    DMAMUX1_Channel1->CCR = DSHOT_DMAMUX_REQ_TX;

    /* 4. Configure Timer for PWM Mode 1 with bit duration */
    TIM4->PSC = 0;
    TIM4->ARR = arr_ticks;
    TIM4->CNT = 0;
    TIM4->CCMR1 = DSHOT_TIM4_CCMR1_TX;

    /* 5. Preload CCR2 with 100% duty (HIGH) to ensure line stays HIGH immediately upon timer start
     */
    TIM4->CCR2 = arr_ticks + 1U;
    TIM4->SR = 0;

    /* 6. Configure DMA for memory-to-peripheral transfer of the full buffer starting at index 0 */
    DMA1_Channel2->CPAR = (uint32_t)&TIM4->CCR2;
    DMA1_Channel2->CMAR = (uint32_t)tx_buf;
    DMA1_Channel2->CNDTR = total_len;

    s_bdshot_transmitting = 1;
    DMA1_Channel2->CCR = DSHOT_DMA_CCR_TX;
    TIM4->DIER = DSHOT_TIM4_DIER_TX;
    TIM4->CCER = DSHOT_TIM4_CCER_TX;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Switch TIM4 CH2 and DMA1 CH2 back to input capture mode for throttle reception
 */
static inline void DShot_SwitchToReceive(void) {
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM4RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM4RST;
    DMA1_Channel2->CCR = 0;
    DMA1->IFCR = DSHOT_DMA_CLEAR_FLAGS;
    DMAMUX1_Channel1->CCR = DSHOT_DMAMUX_REQ_RX;
    DMA1_Channel2->CPAR = (uint32_t)&TIM4->CCR2;
    DMA1_Channel2->CMAR = (uint32_t)s_dma_buf;
    DMA1_Channel2->CNDTR = DSHOT_DMA_BUF_SIZE;
    TIM4->PSC = 0;
    TIM4->ARR = 0xFFFF;
    TIM4->CNT = 0;
    TIM4->CCMR1 = DSHOT_TIM4_CCMR1_RX;
    TIM4->SR = 0;
    s_bdshot_transmitting = 0;
    DMA1_Channel2->CCR = DSHOT_DMA_CCR_RX;
    TIM4->DIER = DSHOT_TIM4_DIER_RX;
    TIM4->CCER = DSHOT_TIM4_CCER_RX;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/*===========================================================================*/
/* Protocol Processing & Decoding                                            */
/*===========================================================================*/

/**
 * @brief Auto-detect DShot baudrate by averaging 14 consecutive bit periods
 * @param buf Pointer to 32 DMA capture timestamps
 * @return Detected DShot speed or DSHOT_SPEED_UNKNOWN
 */
static DShot_Speed_t DShot_AutoDetectSpeed(const uint16_t* buf) {
    uint16_t t_avg = (uint16_t)((buf[30] - buf[0]) / 15U);
    s_tx_arr_ticks = (uint16_t)(((uint32_t)t_avg * 4U) / 5U) - 1U;
    s_threshold = ((uint32_t)t_avg * 9) >> 4;
    float t_avg_us = rx_time_per_tick * (float)t_avg;
    if (t_avg_us >= 0.664e-6f && t_avg_us <= 0.996e-6f) {
        s_tx_arr_ticks = 0;
        return DSHOT_SPEED_1200;
    } else if (t_avg_us >= 1.332e-6f && t_avg_us <= 1.998e-6f) {
        return DSHOT_SPEED_600;
    } else if (t_avg_us >= 2.664e-6f && t_avg_us <= 3.996e-6f) {
        return DSHOT_SPEED_300;
    } else if (t_avg_us >= 5.328e-6f && t_avg_us <= 7.992e-6f) {
        s_tx_arr_ticks = 0;
        return DSHOT_SPEED_150;
    }

    return DSHOT_SPEED_UNKNOWN;
}

/**
 * @brief Pre-fill leading HIGH padding slots in both ping-pong telemetry buffers
 * @param arr_ticks TX Timer auto-reload value
 * @param n_pad Number of leading idle slots
 */
static void DShot_InitPaddingBuffers(uint16_t arr_ticks, uint16_t n_pad) {
    uint16_t high_val = arr_ticks + 1U;
    for (uint16_t b = 0; b < 2; b++) {
        for (uint16_t i = 0; i < n_pad; i++) {
            s_gcr_pwm_buf[b][i] = high_val;
        }
    }
}

/**
 * @brief Decode 32-sample edge buffer into a 16-bit DShot packet
 * @param buf Pointer to 32 DMA capture timestamps
 * @param out_data Decoded 12-bit data payload (throttle + telemetry bit)
 * @param out_is_inverted 1 if inverted BDShot frame, 0 if standard DShot
 * @return 1 on CRC match, 0 on CRC error
 */
static uint8_t DShot_DecodeFrame(const uint16_t* buf, uint16_t* out_data,
                                 uint8_t* out_is_inverted) {
    uint16_t raw_frame = 0;
    const uint32_t* p32 = (const uint32_t*)buf;

    /* Extract 16 bits using 32-bit aligned dual-halfword reads (halves memory load instructions) */
    for (int i = 0; i < 16; i++) {
        uint32_t pair = p32[i];
        uint16_t pulse_len = (uint16_t)((pair >> 16) - (pair & 0xFFFFU));
        raw_frame = (raw_frame << 1) | (pulse_len > s_threshold ? 1U : 0U);
    }

    uint16_t data = raw_frame >> 4;
    uint8_t rx_crc = (uint8_t)(raw_frame & 0x0F);
    uint8_t calc_crc = (uint8_t)((data ^ (data >> 4) ^ (data >> 8)) & 0x0F);

    /* Fast Path: Already locked to a verified signal type */
    if (s_state == DSHOT_STATE_LOCKED) {
        if (s_is_bdshot_locked) {
            if (rx_crc == (uint8_t)((~calc_crc) & 0x0F)) {
                *out_data = data;
                *out_is_inverted = 1;
                return 1;
            }
        } else {
            if (rx_crc == calc_crc) {
                *out_data = data;
                *out_is_inverted = 0;
                return 1;
            }
        }
        return 0; /* CRC mismatch */
    }

    /* Detection Path: Dual-CRC Auto-Sense (runs only for initial 5 frames) */
    if (rx_crc == calc_crc) {
        *out_data = data;
        *out_is_inverted = 0;
        s_bdshot_detect_count = 0;
        if (++s_standard_detect_count >= 5) {
            s_is_bdshot_locked = 0;
            s_state = DSHOT_STATE_LOCKED;
        }
        return 1;
    } else if ((s_speed == DSHOT_SPEED_300 || s_speed == DSHOT_SPEED_600) &&
               rx_crc == (uint8_t)((~calc_crc) & 0x0F)) {
        *out_data = data;
        *out_is_inverted = 1;
        s_standard_detect_count = 0;
        if (++s_bdshot_detect_count >= 5) {
            s_is_bdshot_locked = 1;
            s_state = DSHOT_STATE_LOCKED;
            uint16_t n_pad = (s_speed == DSHOT_SPEED_600) ? 22U : 12U;
            DShot_InitPaddingBuffers(s_tx_arr_ticks, n_pad);
        }
        return 1;
    }

    return 0;
}

/**
 * @brief Map 11-bit raw throttle value to normalized throttle, arming, and direction
 */
static void DShot_MapThrottle(uint16_t raw, uint8_t is_3d, float* out_thr, uint8_t* out_arm,
                              int8_t* out_dir) {
    /* 1. Safety Handshake Interlock: Require 50 consecutive valid zero/neutral frames */
    uint8_t is_handshake_frame = 0;
    if (is_3d) {
        is_handshake_frame = (raw == 0U || raw == DSHOT_3D_DEADBAND);
    } else {
        is_handshake_frame = (raw == 0U);
    }

    if (!s_dshot_armed) {
        if (is_handshake_frame) {
            if (++s_zero_frame_count >= 50U) {
                s_dshot_armed = 1U;
            }
        } else {
            s_zero_frame_count = 0U;
        }
        *out_thr = 0.0f;
        *out_arm = 0U;
        *out_dir = 0;
        return;
    }

    /* 2. Special command range (1 - 47) */
    if (raw > 0U && raw < DSHOT_THROTTLE_MIN) {
        *out_thr = 0.0f;
        *out_arm = 0U;
        *out_dir = 0;
        return;
    }

    /* 3. Operational Throttle & Direction Mapping */
    if (is_3d) {
        if (raw == 0U || raw == DSHOT_3D_DEADBAND) {
            /* 0 = Disarmed by FC, 1048 = Neutral zero-throttle stop */
            *out_thr = 0.0f;
            *out_arm = 0U;
            *out_dir = 0;
        } else if (raw < DSHOT_3D_DEADBAND) {
            /* Reverse throttle: 48 to 1047 -> 1047 is min, 48 is max reverse */
            float norm = (float)(1047U - raw) * DSHOT_3D_REV_INV_RANGE;
            if (norm > 1.0f) norm = 1.0f;
            if (norm < 0.0f) norm = 0.0f;
            *out_thr = norm;
            *out_arm = 1U;
            *out_dir = -1;
        } else {
            /* Forward throttle: 1049 to 2047 -> 1049 is min, 2047 is max forward */
            float norm = (float)(raw - 1049U) * DSHOT_3D_FWD_INV_RANGE;
            if (norm > 1.0f) norm = 1.0f;
            if (norm < 0.0f) norm = 0.0f;
            *out_thr = norm;
            *out_arm = 1U;
            *out_dir = 1;
        }
    } else {
        if (raw == 0U) {
            /* Disarmed / Motor Stop */
            *out_thr = 0.0f;
            *out_arm = 0U;
            *out_dir = 0;
        } else {
            /* Standard 1D throttle: 48 to 2047 */
            float norm = (float)(raw - DSHOT_THROTTLE_MIN) * DSHOT_1D_INV_RANGE;
            if (norm > 1.0f) norm = 1.0f;
            if (norm < 0.0f) norm = 0.0f;
            *out_thr = norm;
            *out_arm = 1U;
            *out_dir = 1;
        }
    }
}

/**
 * @brief Process newly captured DMA frame in interrupt context
 */
static void DShot_ProcessReceivedFrame(void) {
    /* Adaptive per-frame thresholding and speed detection */
    if (!s_is_bdshot_locked) {
        DShot_Speed_t detected = DShot_AutoDetectSpeed(s_dma_buf);
        if (detected == DSHOT_SPEED_UNKNOWN) {
            s_error_count++;
            return;
        }
        s_speed = detected;
    }

    uint16_t data = 0;
    uint8_t is_inverted = 0;
    if (DShot_DecodeFrame(s_dma_buf, &data, &is_inverted)) {
        s_last_frame_tick = FOC_Get_1MhzCounter();
        s_frame_count++;
        s_is_active = 1;

        s_raw_throttle = data >> 1;
        s_telemetry_req = (uint8_t)(data & 0x01U);

        float thr = 0.0f;
        uint8_t arm = 0;
        int8_t dir = 1;
        DShot_MapThrottle(s_raw_throttle, s_dshot_is_3d, &thr, &arm, &dir);

        s_throttle_norm = thr;
        s_arm_req = arm;
        s_dir_req = dir;
    } else {
        s_error_count++;
    }
}

/*===========================================================================*/
/* Interrupt Handler & Driver Interfaces                                     */
/*===========================================================================*/

/* Standard 4B/5B GCR lookup table for nibbles 0x0 to 0xF */
static const uint8_t s_gcr_encode_table[16] = {0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
                                               0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F};

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

    if (FOC_GetState() == FOC_STATE_IDLE || abs_speed < 10.0f) {
        data12 = 0x0FFF; /* Standard Betaflight motor stopped convention */
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
            if (m > 0x1FFU) m = 0x1FFU;
            data12 = (uint16_t)((e << 9) | (m & 0x1FFU));
        }
    }

    /* 4-bit Inverted Checksum */
    uint8_t crc = (uint8_t)((~(data12 ^ (data12 >> 4) ^ (data12 >> 8))) & 0x0F);
    uint16_t packet16 = (uint16_t)((data12 << 4) | crc);

    /* GCR 4B/5B Encoding (20 bits) */
    uint32_t gcr20 = ((uint32_t)s_gcr_encode_table[(packet16 >> 12) & 0x0F] << 15) |
                     ((uint32_t)s_gcr_encode_table[(packet16 >> 8) & 0x0F] << 10) |
                     ((uint32_t)s_gcr_encode_table[(packet16 >> 4) & 0x0F] << 5) |
                     ((uint32_t)s_gcr_encode_table[packet16 & 0x0F]);

    uint16_t bit_period = arr_ticks + 1U;
    uint16_t n_pad = (s_speed == DSHOT_SPEED_600) ? 22U : 12U;
    uint16_t high_val = bit_period;

    /* 1. Start slot: 0% duty (LOW) creates initial falling edge (slots 0..n_pad-1 are pre-filled)
     */
    out_buf[n_pad] = 0;

    /* 2. NRZI wire modulation for 20 GCR bits */
    uint8_t wire_state = 0;
    for (int i = 0; i < 20; i++) {
        uint8_t bit = (uint8_t)((gcr20 >> (19 - i)) & 1U);
        wire_state ^= bit; /* '1' toggles state, '0' maintains state */
        out_buf[n_pad + 1 + i] = wire_state ? high_val : 0;
    }

    /* 3. Trailing idle slot: keep wire at HIGH for seamless handoff back to RX */
    out_buf[n_pad + 21] = high_val;

    return (uint16_t)(n_pad + 22U);
}

/**
 * @brief Main DMA completion callback executing in interrupt context
 */
void DShot_DMA_IRQHandler(void) {
    if (s_bdshot_transmitting) {
        s_tx_frame_count++;
        DShot_SwitchToReceive();
    } else {
        DMA1_Channel2->CCR = 0;

        /* 1. FAST PATH: Launch precomputed telemetry frame IMMEDIATELY (<0.3 us) */
        if (s_is_bdshot_locked && (s_speed == DSHOT_SPEED_300 || s_speed == DSHOT_SPEED_600) &&
            s_precomputed_tx_len > 0) {
            DShot_SwitchToTransmit(s_tx_arr_ticks, s_gcr_pwm_buf[s_active_tx_buf],
                                   s_precomputed_tx_len);
        }

        /* 2. BACKGROUND PATH: Process newly captured frame while TX hardware is transmitting */
        uint32_t start_time = FOC_Get_1MhzCounter();
        DShot_ProcessReceivedFrame();
        dshot_process_time = FOC_Get_1MhzCounter() - start_time;

        /* 3. Prepare telemetry frame for NEXT transaction in the alternate ping-pong buffer */
        if (s_is_bdshot_locked && s_speed >= DSHOT_SPEED_300) {
            uint8_t next_buf_idx = 1U - s_active_tx_buf;
            s_precomputed_tx_len =
                DShot_PrepareTelemetryFrame(s_tx_arr_ticks, s_gcr_pwm_buf[next_buf_idx]);
            s_active_tx_buf = next_buf_idx;
        }

        /* If TX was not triggered, immediately re-arm receiver for next frame */
        if (!s_bdshot_transmitting) {
            DShot_SwitchToReceive();
        }
    }
}

/**
 * @brief Recovery hook executed in interrupt context after a DMA transfer error (TEIF2)
 *
 * Counts the error and re-arms the RX capture. The TIM4 reset and flag clear inside
 * DShot_SwitchToReceive also clear the pending TEIF2, so no extra flag handling is
 * needed on the caller side.
 */
void DShot_DMA_ErrorRecover(void) {
    s_error_count++;
    DShot_SwitchToReceive();
}

/**
 * @brief Initialize DShot hardware peripherals and start DMA reception
 */
static void DShot_Init(void) {
    s_throttle_norm = 0.0f;
    s_arm_req = 0;
    s_dir_req = 0;
    s_is_active = 0;
    s_bdshot_transmitting = 0;
    s_dshot_armed = 0;
    s_zero_frame_count = 0;
    s_active_tx_buf = 0;
    s_precomputed_tx_len = 0;

    FlashConfig_t* cfg = FlashConfig_Get();
    s_dshot_is_3d = (cfg != NULL) ? cfg->dshot_3d_enable : 0U;

    s_speed = DSHOT_SPEED_UNKNOWN;
    s_state = DSHOT_STATE_DETECTING;
    s_threshold = 0;
    s_is_bdshot_locked = 0;
    s_standard_detect_count = 0;
    s_bdshot_detect_count = 0;
    s_last_frame_tick = FOC_Get_1MhzCounter();
    s_raw_throttle = 0;
    s_telemetry_req = 0;

    s_frame_count = 0;
    s_tx_frame_count = 0;
    s_error_count = 0;
    s_timeout_count = 0;
    s_tx_arr_ticks = 0;

    DShot_SwitchToReceive();
    rx_time_per_tick = 1.0f / HAL_RCC_GetSysClockFreq();
}

/**
 * @brief 1 kHz background polling hook for failsafe and arming management
 */
static void DShot_Read(FOC_InputCmd_t* cmd) {
    /* Refresh cached 3D configuration in 1kHz slow task without ISR overhead */
    FlashConfig_t* cfg = FlashConfig_Get();
    if (cfg != NULL) {
        s_dshot_is_3d = cfg->dshot_3d_enable;
    }

    if (s_is_active &&
        ((FOC_Get_1MhzCounter() - s_last_frame_tick) > DSHOT_SIGNAL_TIMEOUT_MS * 1000U)) {
        /* Failsafe timeout reached: disarm immediately */
        s_is_active = 0;
        s_arm_req = 0;
        s_throttle_norm = 0.0f;
        s_dshot_armed = 0;
        s_zero_frame_count = 0;
        s_timeout_count++;

        /* Reset signal polarity and detection state on timeout */
        s_state = DSHOT_STATE_DETECTING;
        s_is_bdshot_locked = 0;
        s_standard_detect_count = 0;
        s_bdshot_detect_count = 0;
        s_speed = DSHOT_SPEED_UNKNOWN;
        s_precomputed_tx_len = 0;
        s_active_tx_buf = 0;
    }

    cmd->throttle = s_throttle_norm;
    cmd->is_active = s_is_active;
    cmd->arm_req = s_arm_req;
    cmd->dir_req = s_dir_req;
}

/**
 * @brief Fast normalized throttle query for 48 kHz HF execution
 */
CCMRAM_FUNC float DShot_ReadFast(void) {
    return s_throttle_norm;
}

/**
 * @brief Get real-time DShot telemetry and diagnostic statistics
 */
void DShot_GetStats(DShot_Stats_t* stats) {
    if (stats == NULL) return;
    stats->speed = s_speed;
    stats->state = s_state;
    stats->is_bdshot_locked = s_is_bdshot_locked;
    stats->is_armed = s_dshot_armed;
    stats->telemetry_req = s_telemetry_req;
    stats->raw_throttle = s_raw_throttle;
    stats->frame_count = s_frame_count;
    stats->tx_frame_count = s_tx_frame_count;
    stats->error_count = s_error_count;
    stats->timeout_count = s_timeout_count;
}

/*===========================================================================*/
/* Pluggable FOC Input Driver Export                                         */
/*===========================================================================*/

const FOC_InputDriver_t g_driver_dshot = {
    .name = "DShot",
    .is_high_speed = 1,
    .init = DShot_Init,
    .read = DShot_Read,
    .read_fast = DShot_ReadFast,
};
