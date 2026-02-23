/**
 * @file uart_cmd.h
 * @brief UART Command Parser for FOC Control
 */

#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>
#include "foc_state_machine.h"

/*===========================================================================*/
/* Configuration                                                             */
/*===========================================================================*/

#define UART_CMD_BUFFER_SIZE     64
#define UART_TX_BUFFER_SIZE      256

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

/**
 * @brief Initialize UART command interface
 */
void UART_CMD_Init(void);

/**
 * @brief Process received UART data (call from UART RX callback)
 * @param data Received byte
 */
void UART_CMD_ProcessByte(uint8_t data);

/**
 * @brief Send real-time data via UART (call from low-priority loop)
 */
void UART_CMD_SendPlotData(void);

/**
 * @brief Send string via UART
 * @param str Null-terminated string
 */
void UART_CMD_SendString(const char *str);

/**
 * @brief Send formatted string via UART
 * @param format Printf-style format string
 */
void UART_CMD_Printf(const char *format, ...);

/**
 * @brief Enable/disable real-time plotting
 * @param enable 1 to enable, 0 to disable
 */
void UART_CMD_EnablePlot(uint8_t enable);

/**
 * @brief Check if plotting is enabled
 * @return 1 if enabled, 0 if disabled
 */
uint8_t UART_CMD_IsPlotEnabled(void);

#endif /* UART_CMD_H */
