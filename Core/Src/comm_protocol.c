/**
 * @file comm_protocol.c
 * @brief Binary communication protocol implementation
 *
 * State-machine packet parser for USB CDC binary protocol.
 * Handles parameter SET/GET, motor control, Flash operations,
 * and optimized binary plot streaming.
 */

#include "comm_protocol.h"

#include <string.h>

#include "bist_profiler.h"
#include "flash_config.h"
#include "foc_state_machine.h"
#include "motor_id.h"
#include "usbd_cdc_if.h"

/*===========================================================================*/
/* Private Types                                                             */
/*===========================================================================*/

/** Parser state machine */
typedef enum {
    PARSE_WAIT_HEADER,
    PARSE_READ_TYPE,
    PARSE_READ_LEN,
    PARSE_READ_PAYLOAD,
    PARSE_READ_CRC
} ParseState_t;

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

static ParseState_t parse_state = PARSE_WAIT_HEADER;
static uint8_t rx_type;
static uint8_t rx_len;
static uint8_t rx_payload[COMM_MAX_PAYLOAD];
static uint8_t rx_idx;

static uint8_t tx_buf[COMM_MAX_PAYLOAD + 4]; /* header + type + len + payload + crc */

#define RX_RING_SIZE 512
static uint8_t rx_ring_buf[RX_RING_SIZE];
static uint16_t rx_ring_head = 0;
static uint16_t rx_ring_tail = 0;

/* Internal parser function (private) */
static void comm_process_byte(uint8_t byte);

/* Removed param_ptrs and param_ro arrays to use type-safe X-Macro accessors */

/*===========================================================================*/
/* Private Functions                                                         */
/*===========================================================================*/

/**
 * @brief Compute XOR checksum over buffer
 */
static uint8_t compute_crc(const uint8_t* data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

/**
 * @brief Send a binary packet via USB CDC
 * @param type   Packet type
 * @param payload Payload data
 * @param len    Payload length
 */
static void send_packet(uint8_t type, const uint8_t* payload, uint8_t len) {
    /* Safety: Never exceed protocol length limit or buffer size */
    if (len > COMM_MAX_PAYLOAD) return;

    /* Wait for previous transmission to complete BEFORE modifying the shared tx_buf */
    uint32_t wait_timeout = 500000;
    while (CDC_IsTxBusy()) {
        if (--wait_timeout == 0) return;
    }

    tx_buf[0] = COMM_HEADER;
    tx_buf[1] = type;
    tx_buf[2] = len;
    if (len > 0 && payload) {
        memcpy(&tx_buf[3], payload, len);
    }
    /* CRC over type + len + payload */
    tx_buf[3 + len] = compute_crc(&tx_buf[1], 2 + len);

    /* Start transmission (should return USBD_OK immediately since we waited) */
    CDC_Transmit_FS(tx_buf, 4 + len);
}

static float get_param_value(uint8_t pid) {
    FlashConfig_t* cfg = FlashConfig_Get();
    switch (pid) {
#define PARAM_DEF(id, type, name, default_val) \
    case id:                                   \
        return (float)cfg->name;
#include "param_table.def"
        case PID_SPD_REF:
            return g_foc.cmd.speed_ref_target;
        case PID_TRQ_REF:
            return g_foc.cmd.Iq_ref_target;
        case PID_VBUS:
            return g_foc.data.Vbus;
        case PID_RPM:
            return g_foc.data.speed_rpm;
        case PID_ID_MEAS:
            return g_foc.data.Id;
        case PID_IQ_MEAS:
            return g_foc.data.Iq;
        case PID_IA:
            return g_foc.data.Ia;
        case PID_IB:
            return g_foc.data.Ib;
        case PID_ID_RS_MEAS: {
            MotorID_Result_t res;
            MotorID_GetResults(&res);
            return res.measured_rs;
        }
        case PID_ID_LS_MEAS: {
            MotorID_Result_t res;
            MotorID_GetResults(&res);
            return res.measured_ls;
        }
        default:
            return 0.0f;
    }
}

static void set_param_value(uint8_t pid, float val) {
    FlashConfig_t* cfg = FlashConfig_Get();
    switch (pid) {
#define PARAM_DEF(id, type, name, default_val) \
    case id:                                   \
        cfg->name = (type)val;                 \
        break;
#include "param_table.def"
        case PID_SPD_REF:
            g_foc.cmd.speed_ref_target = val;
            break;
        case PID_TRQ_REF:
            g_foc.cmd.Iq_ref_target = val;
            break;
        default:
            break;
    }
}

static uint8_t is_param_ro(uint8_t pid) {
    return (pid >= PID_SPD_REF);
}

static uint8_t is_pid_defined(uint8_t pid) {
    if (pid >= PID_SPD_REF && pid < PID_COUNT) return 1;
    switch (pid) {
#define PARAM_DEF(id, type, name, default_val) \
    case id:                                   \
        return 1;
#include "param_table.def"
        case PID_ID_RS_MEAS:
        case PID_ID_LS_MEAS:
            return 1;
        default:
            return 0;
    }
}

/**
 * @brief Send ACK/NACK response
 */
static void send_ack(uint8_t cmd_type, uint8_t status) {
    uint8_t payload[2] = {cmd_type, status};
    send_packet(RSP_ACK, payload, 2);
}

/**
 * @brief Handle a complete received packet
 */
static void handle_packet(uint8_t type, uint8_t* payload, uint8_t len) {
    FlashConfig_t* cfg = FlashConfig_Get();

    switch (type) {
        case CMD_SET: {
            if (len < 5) {
                send_ack(type, 1);
                break;
            }
            uint8_t pid = payload[0];
            if (pid >= PID_COUNT || is_param_ro(pid)) {
                send_ack(type, 1);
                break;
            }

            float val;
            memcpy(&val, &payload[1], 4);
            set_param_value(pid, val);

            /* Apply config params to live system */
            if (pid < PID_SPD_REF) {
                FlashConfig_Apply();
            }
            send_ack(type, 0);
            break;
        }

        case CMD_GET: {
            if (len < 1) {
                send_ack(type, 1);
                break;
            }
            uint8_t pid = payload[0];
            if (pid >= PID_COUNT) {
                send_ack(type, 1);
                break;
            }

            uint8_t rsp[5];
            rsp[0] = pid;
            float val = get_param_value(pid);
            memcpy(&rsp[1], &val, 4);
            send_packet(RSP_VALUE, rsp, 5);
            break;
        }

        case CMD_SAVE:
            send_ack(type, (FlashConfig_Save() == 0) ? 0 : 1);
            break;

        case CMD_LOAD:
            FlashConfig_Init();
            FlashConfig_Apply();
            send_ack(type, 0);
            break;

        case CMD_DEFAULTS:
            FlashConfig_LoadDefaults();
            FlashConfig_Apply();
            send_ack(type, 0);
            break;

        case CMD_START:
            FOC_Start();
            send_ack(type, 0);
            break;

        case CMD_STOP:
            FOC_Stop();
            send_ack(type, 0);
            break;

        case CMD_DIR: {
            if (len < 1) {
                send_ack(type, 1);
                break;
            }
            if (FOC_GetState() != FOC_STATE_IDLE && FOC_GetState() != FOC_STATE_STOP &&
                FOC_GetState() != FOC_STATE_FAULT) {
                send_ack(type, 1);
                break;
            }
            int8_t dir = (payload[0] != 0) ? -1 : 1;
            FOC_SetDirection(dir);
            cfg->direction = dir;
            send_ack(type, 0);
            break;
        }

        case CMD_SPEED: {
            if (len < 4) {
                send_ack(type, 1);
                break;
            }
            float rpm;
            memcpy(&rpm, payload, 4);
            FOC_SetControlMode(FOC_MODE_SPEED);
            FOC_SetSpeedRef(rpm);
            send_ack(type, 0);
            break;
        }

        case CMD_TORQUE: {
            if (len < 4) {
                send_ack(type, 1);
                break;
            }
            float pct;
            memcpy(&pct, payload, 4);
            FOC_SetControlMode(FOC_MODE_TORQUE);
            FOC_SetTorqueRef(pct);
            send_ack(type, 0);
            break;
        }

        case CMD_PLOT: {
            if (len < 1) {
                send_ack(type, 1);
                break;
            }
            g_foc.plot.enabled = payload[0];
            send_ack(type, 0);
            break;
        }

        case CMD_STATUS: {
            uint8_t rsp[12];
            rsp[0] = (uint8_t)g_foc.status.state;
            rsp[1] = (uint8_t)g_foc.status.fault;
            rsp[2] = (FOC_GetDirection() < 0) ? 1 : 0;
            rsp[3] = 0; /* padding */
            memcpy(&rsp[4], &g_foc.data.speed_rpm, 4);
            memcpy(&rsp[8], &g_foc.data.Vbus, 4);
            send_packet(RSP_STATUS, rsp, 12);
            break;
        }

        case CMD_PARAM_ALL: {
            /*
             * PC wants all parameters.
             * As of now we have 60+ parameters (~310 bytes).
             * Protocol limit is 255 bytes per packet.
             * We split the response into batches of 32 parameters.
             */
            uint8_t rsp[1 + 32 * 5];
            uint8_t count = 0;
            uint8_t pos = 1;

            for (uint8_t i = 0; i < PID_COUNT; i++) {
                if (!is_pid_defined(i)) continue;

                rsp[pos++] = i;
                float val = get_param_value(i);
                memcpy(&rsp[pos], &val, 4);
                pos += 4;
                count++;

                /* If we filled a batch, send it and start a new one */
                if (count >= 32) {
                    rsp[0] = count;
                    send_packet(RSP_PARAM_ALL, rsp, pos);

                    /* Reset for next batch */
                    count = 0;
                    pos = 1;
                }
            }

            /* Send remaining parameters in the last batch */
            if (count > 0) {
                rsp[0] = count;
                send_packet(RSP_PARAM_ALL, rsp, pos);
            }
            break;
        }

        case CMD_IDENT:
            if (FOC_GetState() != FOC_STATE_IDLE) {
                send_ack(type, 1);
                break;
            }
            FOC_StartSelfCommission();
            send_ack(type, 0);
            break;

        case CMD_CLEAR:
            FOC_ClearFault();
            send_ack(type, 0);
            break;

        case CMD_BIST: {
            if (len >= 13) {
                uint8_t mode = payload[0];
                float amp_args, offset_args, freq_args;
                memcpy(&amp_args, &payload[1], 4);
                memcpy(&offset_args, &payload[5], 4);
                memcpy(&freq_args, &payload[9], 4);
                BIST_Start(&g_foc.ctrl.bist, mode, amp_args, offset_args, freq_args);
                /* If starting test, switch to torque mode automatically */
                if (mode != BIST_MODE_OFF) {
                    FOC_SetControlMode(FOC_MODE_TORQUE);
                }
                send_ack(type, 0);
            } else {
                send_ack(type, 1);
            }
            break;
        }

        default:
            send_ack(type, 1);
            break;
    }
}

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

void Comm_Init(void) {
    parse_state = PARSE_WAIT_HEADER;
    rx_idx = 0;
    rx_ring_head = 0;
    rx_ring_tail = 0;
}

void Comm_PushByte(uint8_t byte) {
    uint16_t next = (rx_ring_head + 1) % RX_RING_SIZE;
    if (next != rx_ring_tail) {
        rx_ring_buf[rx_ring_head] = byte;
        rx_ring_head = next;
    }
}

void Comm_Update(void) {
    while (rx_ring_tail != rx_ring_head) {
        uint8_t byte = rx_ring_buf[rx_ring_tail];
        rx_ring_tail = (rx_ring_tail + 1) % RX_RING_SIZE;
        comm_process_byte(byte);
    }
}

static void comm_process_byte(uint8_t byte) {
    switch (parse_state) {
        case PARSE_WAIT_HEADER:
            if (byte == COMM_HEADER) {
                parse_state = PARSE_READ_TYPE;
            }
            break;

        case PARSE_READ_TYPE:
            rx_type = byte;
            parse_state = PARSE_READ_LEN;
            break;

        case PARSE_READ_LEN:
            rx_len = byte;
            rx_idx = 0;
            if (rx_len == 0) {
                parse_state = PARSE_READ_CRC;
            } else if (rx_len > COMM_MAX_PAYLOAD) {
                parse_state = PARSE_WAIT_HEADER; /* invalid, reset */
            } else {
                parse_state = PARSE_READ_PAYLOAD;
            }
            break;

        case PARSE_READ_PAYLOAD:
            rx_payload[rx_idx++] = byte;
            if (rx_idx >= rx_len) {
                parse_state = PARSE_READ_CRC;
            }
            break;

        case PARSE_READ_CRC: {
            /* Verify CRC: XOR of type + len + payload */
            uint8_t crc = rx_type ^ rx_len;
            for (uint8_t i = 0; i < rx_len; i++) {
                crc ^= rx_payload[i];
            }
            if (crc == byte) {
                handle_packet(rx_type, rx_payload, rx_len);
            }
            /* else: corrupted packet, silently drop */
            parse_state = PARSE_WAIT_HEADER;
            break;
        }
    }
}

void Comm_SendPlotPacket(void) {
    uint8_t payload[24]; /* 6 × float = 24 bytes */
    memcpy(&payload[0], &g_foc.plot.Vd, 4);
    memcpy(&payload[4], &g_foc.plot.Vq, 4);
    memcpy(&payload[8], &g_foc.plot.Id, 4);
    memcpy(&payload[12], &g_foc.plot.Iq, 4);
    memcpy(&payload[16], &g_foc.plot.Iq_ref, 4);
    memcpy(&payload[20], &g_foc.plot.theta_elec, 4);
    send_packet(RSP_PLOT, payload, 24);
}
