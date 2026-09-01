#include "foc_input.h"

#include "foc_config.h"
#include "foc_state_machine.h"
#include "peripheral_init.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Biến trạng thái nội bộ đóng gói */
static FOC_InputSource_t s_input_source = FOC_INPUT_SOURCE_NONE;
static uint16_t s_pot_raw = 0;
static float s_pot_val = 0.0f;
static float s_pot_raw_filt = 0.0f;
static float s_vq_min_active = 0.05f;
static uint8_t s_pot_unlocked = 0;
static uint16_t s_pot_zero_count = 0;

static void FOC_Input_ProcessPot(void);
static void FOC_Input_ProcessUART(void);

void FOC_Input_Init(void) {
    s_pot_unlocked = 0;
    s_pot_zero_count = 0;
    s_input_source = FOC_INPUT_SOURCE_NONE;
    s_vq_min_active = 0.05f;
    s_pot_raw = ADC_ReadPot_SingleShot();
    s_pot_raw_filt = (float)s_pot_raw;
    s_pot_val = s_pot_raw_filt / 4095.0f;
}

void FOC_Input_SetMinVq(float vq_min) {
    s_vq_min_active = vq_min;
}

FOC_InputSource_t FOC_Input_GetSource(void) {
    return s_input_source;
}

void FOC_Input_Update(void) {
    // 1. Phân chia quyền ưu tiên: USB cắm -> ngắt Potentiometer
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        s_input_source = FOC_INPUT_SOURCE_UART_USB;
    } else {
        s_input_source = FOC_INPUT_SOURCE_POT;
    }

    // 2. Chạy hàm xử lý nguồn đầu vào tương ứng
    switch (s_input_source) {
        case FOC_INPUT_SOURCE_POT:
            FOC_Input_ProcessPot();
            break;

        case FOC_INPUT_SOURCE_UART_USB:
            FOC_Input_ProcessUART();
            break;

        default:
            break;
    }
}

static void FOC_Input_ProcessPot(void) {
    s_pot_raw = ADC_ReadPot_SingleShot();
    s_pot_raw_filt = POT_LPF_ALPHA * s_pot_raw_filt + (1.0f - POT_LPF_ALPHA) * (float)s_pot_raw;
    s_pot_val = s_pot_raw_filt / 4095.0f;

    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_FAULT) {
        s_vq_min_active = 0.05f;
        if (g_foc.status.state == FOC_STATE_FAULT) {
            s_pot_unlocked = 0;
            s_pot_zero_count = 0;
        }
        if (!s_pot_unlocked) {
            if (s_pot_raw_filt < (float)POT_STOP_THRESHOLD) {
                s_pot_zero_count++;
                if (s_pot_zero_count >= 100) {
                    s_pot_unlocked = 1;
                }
            } else {
                s_pot_zero_count = 0;
            }
        }
    }

    if (g_foc.status.state == FOC_STATE_IDLE && s_pot_unlocked &&
        s_pot_raw_filt > (float)POT_START_THRESHOLD) {
        FOC_SetControlMode(FOC_MODE_VOLTAGE);
        FOC_Start();
    }

    if (g_foc.status.state != FOC_STATE_IDLE && g_foc.status.state != FOC_STATE_FAULT &&
        g_foc.status.state != FOC_STATE_STOP) {
        if (s_pot_raw_filt < (float)POT_STOP_THRESHOLD) {
            FOC_SetVoltageRef(0.0f);
            FOC_Stop();
            s_pot_unlocked = 0;
            s_pot_zero_count = 0;
            s_vq_min_active = 0.05f;
        }
    }

    if (g_foc.status.state == FOC_STATE_RUN) {
        float norm_val = (s_pot_raw_filt - (float)POT_STOP_THRESHOLD) /
                         ((float)POT_ADC_MAX - (float)POT_STOP_THRESHOLD);
        norm_val = clampf(norm_val, 0.0f, 1.0f);

        float vq_min = s_vq_min_active;
        FOC_SetVoltageRef((vq_min + norm_val * (1.0f - vq_min)) * 100.0f);
    }
}

static void FOC_Input_ProcessUART(void) {
    s_pot_unlocked = 0;
    s_pot_zero_count = 0;
    s_pot_raw = 0;
    s_pot_val = 0.0f;
    s_pot_raw_filt = 0.0f;
    s_vq_min_active = 0.05f;
}
