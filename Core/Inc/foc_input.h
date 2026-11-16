#ifndef FOC_INPUT_H
#define FOC_INPUT_H

#include <stdint.h>

#include "foc_config.h"

/*===========================================================================*/
/* Input Data Structures                                                     */
/*===========================================================================*/

typedef enum {
    FOC_INPUT_SOURCE_NONE = 0, /**< Hardware input disabled (Host GUI / USB control only) */
    FOC_INPUT_SOURCE_POT,      /**< Onboard Potentiometer (PC4) */
    FOC_INPUT_SOURCE_DSHOT,    /**< DShot digital protocol input*/
} FOC_InputSource_t;

/**
 * @brief Standardized normalized command produced by any physical input driver
 */
typedef struct {
    uint8_t is_active; /**< 1 if driver hardware is active and valid, 0 if disconnected/failsafe */
    uint8_t arm_req;   /**< 1 if requesting motor run, 0 if requesting motor stop */
    uint8_t is_armed_ready; /**< 1 if startup zero-throttle arming hold has latched */
    uint8_t beacon_req;     /**< Non-zero (1..5) when DShot Beacon command is requested */
} FOC_InputCmd_t;

/**
 * @brief Input Driver interface for pluggable controllers
 */
typedef struct {
    const char* name;
    void (*init)(void);
    void (*deinit)(void);
    void (*update)(FOC_InputCmd_t* cmd);
    float (*read_throttle)(void);
} FOC_InputDriver_t;

/*===========================================================================*/
/* Public APIs                                                               */
/*===========================================================================*/

void FOC_Input_Init(void);
void FOC_Input_Update(void);
void FOC_Input_PushThrottle(float thr);
void FOC_Input_SelectSource(FOC_InputSource_t source);
void FOC_Input_SetMinVq(float vq_min);
void FOC_Input_NotifyHostActivity(uint16_t timeout_ms);
uint8_t FOC_Input_IsHostControlling(void);

FOC_InputSource_t FOC_Input_GetSource(void);
float FOC_Input_GetThrottle(void);
uint8_t FOC_Input_IsArmed(void);

#endif /* FOC_INPUT_H */
