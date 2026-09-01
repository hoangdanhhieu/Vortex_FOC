#ifndef FOC_INPUT_H
#define FOC_INPUT_H

#include "main.h"

typedef enum {
    FOC_INPUT_SOURCE_NONE = 0,
    FOC_INPUT_SOURCE_POT,       /**< Potentiometer PC4 */
    FOC_INPUT_SOURCE_UART_USB,   /**< Host GUI via USB Virtual COM Port */
} FOC_InputSource_t;

void FOC_Input_Init(void);
void FOC_Input_Update(void);
void FOC_Input_SetMinVq(float vq_min);
FOC_InputSource_t FOC_Input_GetSource(void);

#endif /* FOC_INPUT_H */
