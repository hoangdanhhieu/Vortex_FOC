/**
 * @file input_dshot.h
 * @brief High-performance DShot receiver and telemetry driver for Vortex FOC
 *
 * Implements DShot 150/300/600/1200 decoding, automatic baudrate detection,
 * self-aligning DMA capture, 3D throttle mode, and Bidirectional DShot (BDShot)
 * telemetry support on PB7 (TIM4 CH2).
 */

#ifndef INPUT_DSHOT_H
#define INPUT_DSHOT_H

#include <stdint.h>

#include "foc_input.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* DShot Enumerations & Structures                                           */
/*===========================================================================*/

typedef enum {
    DSHOT_SPEED_UNKNOWN = 0,
    DSHOT_SPEED_150,
    DSHOT_SPEED_300,
    DSHOT_SPEED_600,
    DSHOT_SPEED_1200,
} DShot_Speed_t;

typedef enum {
    DSHOT_STATE_DETECTING = 0, /**< Auto-detecting baudrate and signal polarity */
    DSHOT_STATE_LOCKED,        /**< Baudrate and polarity locked, decoding active */
} DShot_State_t;

typedef struct {
    DShot_Speed_t speed;      /**< Detected DShot speed */
    DShot_State_t state;      /**< Detection and synchronization state */
    uint8_t is_bdshot_locked; /**< 1 = Inverted BDShot protocol locked, 0 = Standard */
    uint8_t is_armed;         /**< 1 = Handshake completed and driver armed, 0 = Unarmed */
    uint8_t telemetry_req;    /**< Telemetry request bit from last valid frame */
    uint16_t raw_throttle;    /**< Raw 11-bit throttle value (0 - 2047) */
    uint32_t frame_count;     /**< Total valid frames received */
    uint32_t tx_frame_count;  /**< Total BDShot telemetry frames transmitted */
    uint32_t error_count;     /**< CRC error count */
    uint32_t timeout_count;   /**< Failsafe timeout events */
} DShot_Stats_t;

/*===========================================================================*/
/* Public Interfaces & Driver Hook                                           */
/*===========================================================================*/

extern const FOC_InputDriver_t g_driver_dshot;

/**
 * @brief Interrupt service routine callback invoked from DMA1_Channel2_IRQHandler
 */
void DShot_DMA_IRQHandler(void);

/**
 * @brief Recovery callback invoked from DMA1_Channel2_IRQHandler on a DMA transfer error
 */
void DShot_DMA_ErrorRecover(void);

/**
 * @brief Fast normalized throttle query for 48 kHz HF execution
 * @return Normalized throttle value (0.0f to 1.0f)
 */
CCMRAM_FUNC float DShot_ReadFast(void);

/**
 * @brief Get real-time DShot telemetry and diagnostic statistics
 * @param stats Pointer to output statistics structure
 */
void DShot_GetStats(DShot_Stats_t* stats);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_DSHOT_H */
