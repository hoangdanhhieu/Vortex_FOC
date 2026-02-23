/**
 * @file comm_protocol.h
 * @brief Binary communication protocol for FOC GUI configurator
 *
 * Replaces text-based UART commands with a compact binary protocol
 * optimized for real-time streaming and GUI interaction over USB CDC.
 */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdint.h>

/*===========================================================================*/
/* Protocol Constants                                                        */
/*===========================================================================*/

#define COMM_HEADER 0xAA
#define COMM_MAX_PAYLOAD 255

/** Streaming decimation: 48kHz / 48 = 1kHz plot rate */
#define PLOT_DECIMATION 48

/*===========================================================================*/
/* Command Types (PC → MCU)                                                  */
/*===========================================================================*/

#define CMD_SET 0x01       /**< Set parameter: id(1B) + value(4B) */
#define CMD_GET 0x02       /**< Get parameter: id(1B) */
#define CMD_SAVE 0x03      /**< Save config to Flash */
#define CMD_LOAD 0x04      /**< Load config from Flash */
#define CMD_DEFAULTS 0x05  /**< Reset to defaults */
#define CMD_START 0x06     /**< Start motor */
#define CMD_STOP 0x07      /**< Stop motor */
#define CMD_DIR 0x08       /**< Set direction: dir(1B) 0=FWD,1=REV */
#define CMD_SPEED 0x09     /**< Set speed ref: rpm(4B float) */
#define CMD_TORQUE 0x0A    /**< Set torque ref: pct(4B float) */
#define CMD_PLOT 0x0B      /**< Enable/disable plot: enable(1B) */
#define CMD_STATUS 0x0C    /**< Request status */
#define CMD_PARAM_ALL 0x0D /**< Request all parameters */
#define CMD_IDENT 0x0E     /**< Trigger motor parameter identification */
#define CMD_CLEAR 0x0F     /**< Clear faults */
#define CMD_BIST 0x10      /**< Built-In Self Test Profiler settings */

/*===========================================================================*/
/* Response Types (MCU → PC)                                                 */
/*===========================================================================*/

#define RSP_ACK 0x81       /**< ACK: cmd_type(1B) + status(1B) */
#define RSP_VALUE 0x82     /**< Value: id(1B) + value(4B float) */
#define RSP_STATUS 0x83    /**< Status: state(1B)+fault(1B)+dir(1B)+pad+rpm(4B)+vbus(4B) */
#define RSP_PARAM_ALL 0x84 /**< All params: count(1B) + [id(1B)+val(4B)]×N */
#define RSP_PLOT 0x90      /**< Stream: Ia(4B)+Ib(4B)+Id(4B)+Iq(4B)+theta(4B)+rpm(4B) */

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
    PID_ID_RS_MEAS,
    PID_ID_LS_MEAS,
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
 * @brief Process queued bytes and execute protocol logic (Call from main loop)
 */
void Comm_Update(void);

/**
 * @brief Send plot streaming packet from snapshot data
 */
void Comm_SendPlotPacket(void);

/**
 * @brief Send status packet
 */
void Comm_SendStatus(void);

#endif /* COMM_PROTOCOL_H */
