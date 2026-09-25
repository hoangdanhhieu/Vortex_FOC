/**
 * @file comm_protocol.h
 * @brief Binary communication protocol for FOC GUI configurator
 *
 * Replaces text-based UART commands with a compact binary protocol
 * optimized for real-time continuous streaming over USB CDC.
 */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdint.h>

/*===========================================================================*/
/* Protocol Constants                                                        */
/*===========================================================================*/

#define COMM_HEADER 0xAA
#define COMM_MAX_PAYLOAD 255

/*===========================================================================*/
/* Command Types (PC → MCU)                                                  */
/*===========================================================================*/

#define CMD_SET 0x01           /**< Set parameter: id(1B) + value(4B) */
#define CMD_GET 0x02           /**< Get parameter: id(1B) */
#define CMD_SAVE 0x03          /**< Save config to Flash */
#define CMD_LOAD 0x04          /**< Load config from Flash */
#define CMD_DEFAULTS 0x05      /**< Reset to defaults */
#define CMD_START 0x06         /**< Start motor */
#define CMD_STOP 0x07          /**< Stop motor */
#define CMD_DIR 0x08           /**< Set direction: dir(1B) 0=FWD,1=REV */
#define CMD_SPEED 0x09         /**< Set speed ref: speed_rad(4B float) */
#define CMD_TORQUE 0x0A        /**< Set torque ref: pct(4B float) */
#define CMD_PLOT 0x0B          /**< (reserved) */
#define CMD_STATUS 0x0C        /**< Request status */
#define CMD_PARAM_ALL 0x0D     /**< Request all parameters */
#define CMD_IDENT 0x0E         /**< Trigger motor parameter identification */
#define CMD_CLEAR 0x0F         /**< Clear faults */
#define CMD_PROFILER 0x10      /**< Dynamic Response Profiler settings */
#define CMD_VOLTAGE 0x11       /**< Set voltage ref: pct(4B float) */
#define CMD_IDENT_FLUX 0x12    /**< Trigger offline flux identification */
#define CMD_IDENT_INERTIA 0x13 /**< Trigger offline inertia identification */
#define CMD_STREAM_START 0x14  /**< Start continuous stream: num_ch(1B) + dec(1B) + ch[0..N-1] */
#define CMD_STREAM_STOP 0x15   /**< Stop continuous stream */

/*===========================================================================*/
/* Response Types (MCU → PC)                                                 */
/*===========================================================================*/

#define RSP_ACK 0x81         /**< ACK: cmd_type(1B) + status(1B) */
#define RSP_VALUE 0x82       /**< Value: id(1B) + value(4B float) */
#define RSP_STATUS 0x83      /**< Status: state+fault+dir+pad+omega_elec+vbus+ibus */
#define RSP_PARAM_ALL 0x84   /**< All params: count(1B) + [id(1B)+val(4B)]xN */
#define RSP_STREAM_DATA 0x91 /**< Stream: seq(2B) + num_sets(1B) + [ch0..ch3 fp16] x num_sets */

/*===========================================================================*/
/* Streaming Constants                                                       */
/*===========================================================================*/

/** Decimation range: effective rate = 48000 / decimation
 *  dec=1 → 48000 Hz (max, ~375 KB/s USB load)
 *  dec=10 → 4800 Hz (min, ~38 KB/s USB load)
 */
#define STREAM_DEC_MIN 1
#define STREAM_DEC_MAX 10

/*===========================================================================*/
/* Parameter IDs                                                             */
/*===========================================================================*/

enum {
/* Auto-generate Configurable Parameter Enums from X-Macro table */
#define PARAM_DEF(id, type, name, default_val) id,
#include "param_table.def"

    /* Live params (read-only via SET) */
    PID_SPD_REF,
    PID_TRQ_REF,
    PID_VBUS,
    PID_RPM,
    PID_ID_MEAS,
    PID_IQ_MEAS,
    PID_IA,
    PID_IB,
    PID_IC,
    PID_DUTY_A,
    PID_DUTY_B,
    PID_DUTY_C,
    PID_VD,
    PID_VQ,
    PID_ID_RS_MEAS,
    PID_ID_LS_MEAS,
    PID_ID_ISAT_MEAS,
    PID_ID_ALPHA_MEAS,
    PID_ID_DT_MEAS,
    PID_ID_FREQ_MEAS,
    PID_ID_FLUX_MEAS,
    PID_ID_KV_MEAS,
    PID_ID_INERTIA_MEAS,
    PID_ID_B0_MEAS,
    PID_USER_PLOT1,
    PID_USER_PLOT2,
    PID_USER_PLOT3,
    PID_THETA_ELEC,
    PID_COUNT
};

/*===========================================================================*/
/* Public API                                                                */
/*===========================================================================*/

/**
 * @brief Initialize communication protocol
 */
void Comm_Init(void);

/**
 * @brief Push a received byte into the ring buffer (ISR safe)
 * @param byte Received byte
 */
void Comm_PushByte(uint8_t byte);

/**
 * @brief Process queued RX bytes and execute protocol logic (call from main loop)
 */
void Comm_Update(void);

/**
 * @brief Push one sample-set into the streaming ring buffer.
 *        Must be called from the HF ISR. Handles decimation internally.
 *        No-op when stream_active == 0.
 */
#include "foc_config.h"
CCMRAM_FUNC void Comm_StreamPush(void);

/**
 * @brief Drain the streaming ring buffer and transmit over USB CDC.
 *        Called from Comm_Update() in Thread Mode (main superloop).
 */
void Comm_StreamDrain(void);

/**
 * @brief Send status packet (called periodically from slow task)
 */
void Comm_SendStatus(void);

/** Non-zero while streaming is active */
extern volatile uint8_t stream_active;

#endif /* COMM_PROTOCOL_H */
