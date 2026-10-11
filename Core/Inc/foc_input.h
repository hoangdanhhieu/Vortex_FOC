#ifndef FOC_INPUT_H
#define FOC_INPUT_H

#include <stdint.h>

#include "foc_config.h"

/*===========================================================================*/
/* Input Data Structures                                                     */
/*===========================================================================*/

typedef enum {
    FOC_INPUT_SOURCE_NONE = 0,
    FOC_INPUT_SOURCE_POT,      /**< Onboard Potentiometer (PC4) */
    FOC_INPUT_SOURCE_CUSTOM,   /**< Pluggable external/custom driver */
    FOC_INPUT_SOURCE_UART_USB, /**< Host GUI via USB Virtual COM Port (Override) */
    FOC_INPUT_SOURCE_DSHOT,    /**< DShot digital protocol input (PB7 / TIM4_CH2) */
} FOC_InputSource_t;

/**
 * @brief Standardized normalized command produced by any physical input driver
 */
typedef struct {
    float throttle;    /**< Normalized command: 0.0f to 1.0f */
    uint8_t is_active; /**< 1 if driver hardware is active and valid, 0 if disconnected/failsafe */
    uint8_t arm_req;   /**< 1 if requesting motor run, 0 if requesting motor stop */
    int8_t dir_req;    /**< Direction: +1 = FWD, -1 = REV, 0 = Keep current.
                             Consumed by the input layer at 1 kHz: only changeable while
                             stopped; a mismatch while running stops the motor and it
                             restarts in the requested direction shortly after. */
} FOC_InputCmd_t;

/**
 * @brief Input Driver interface for pluggable controllers
 */
typedef struct {
    const char* name;
    uint8_t is_high_speed; /**< 1 if driver supports high-speed streaming / ISR ingestion */
    void (*init)(void);
    void (*read)(FOC_InputCmd_t* cmd);
    float (*read_fast)(
        void); /**< Optional fast hook returning normalized throttle (0.0f to 1.0f) */
} FOC_InputDriver_t;

/*===========================================================================*/
/* Public APIs                                                               */
/*===========================================================================*/

void FOC_Input_Init(void);
void FOC_Input_Update(void);
CCMRAM_FUNC void FOC_Input_Update_HF(void);
void FOC_Input_SelectSource(FOC_InputSource_t source);
void FOC_Input_RegisterCustomDriver(const FOC_InputDriver_t* driver);
void FOC_Input_SetMinVq(float vq_min);
void FOC_Input_NotifyHostActivity(uint16_t timeout_ms);
uint8_t FOC_Input_IsHostControlling(void);

FOC_InputSource_t FOC_Input_GetSource(void);
float FOC_Input_GetThrottle(void);
uint8_t FOC_Input_IsArmed(void);

#endif /* FOC_INPUT_H */
