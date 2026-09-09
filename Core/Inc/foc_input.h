#ifndef FOC_INPUT_H
#define FOC_INPUT_H

#include <stdint.h>
#include "foc_config.h"

/*===========================================================================*/
/* Input Data Structures                                                     */
/*===========================================================================*/

typedef enum {
    FOC_INPUT_SOURCE_NONE = 0,
    FOC_INPUT_SOURCE_POT,       /**< Onboard Potentiometer (PC4) */
    FOC_INPUT_SOURCE_CUSTOM,    /**< Pluggable external/custom driver */
    FOC_INPUT_SOURCE_UART_USB,  /**< Host GUI via USB Virtual COM Port (Override) */
} FOC_InputSource_t;

/**
 * @brief Standardized normalized command produced by any physical input driver
 */
typedef struct {
    float throttle;    /**< Normalized command: 0.0f to 1.0f */
    uint8_t is_active; /**< 1 if driver hardware is active and valid, 0 if disconnected/failsafe */
    uint8_t arm_req;   /**< 1 if requesting motor run, 0 if requesting motor stop */
    int8_t dir_req;    /**< Direction: +1 = FWD, -1 = REV, 0 = Keep current */
} FOC_InputCmd_t;

/**
 * @brief Input Driver interface for pluggable controllers
 */
typedef struct {
    const char* name;
    void (*init)(void);
    void (*read)(FOC_InputCmd_t* cmd);
} FOC_InputDriver_t;

/*===========================================================================*/
/* Public APIs                                                               */
/*===========================================================================*/

void FOC_Input_Init(void);
void FOC_Input_Update(void);
void FOC_Input_SelectSource(FOC_InputSource_t source);
void FOC_Input_RegisterCustomDriver(const FOC_InputDriver_t* driver);
void FOC_Input_SetMinVq(float vq_min);

FOC_InputSource_t FOC_Input_GetSource(void);
float FOC_Input_GetThrottle(void);
uint8_t FOC_Input_IsArmed(void);

#endif /* FOC_INPUT_H */
