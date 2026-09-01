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
#include "foc_startup.h"
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
static uint8_t rx_payload[COMM_MAX_PAYLOAD] __attribute__((aligned(4)));
static uint8_t rx_idx;

static uint8_t tx_buf[COMM_MAX_PAYLOAD + 4]
    __attribute__((aligned(4))); /* header + type + len + payload + crc */

#define PLOT_CHANNELS 7
#define PLOT_BATCH_SIZE 16
static int16_t plot_batch_buf[PLOT_BATCH_SIZE * PLOT_CHANNELS] __attribute__((aligned(4)));

#define PLOT_RING_SIZE 256
#define PLOT_RING_MASK (PLOT_RING_SIZE - 1)
static int16_t plot_ring_buf[PLOT_RING_SIZE * PLOT_CHANNELS] __attribute__((aligned(4)));
static volatile uint16_t plot_ring_head = 0;
static volatile uint16_t plot_ring_tail = 0;

#define RX_RING_SIZE 512
static uint8_t rx_ring_buf[RX_RING_SIZE] __attribute__((aligned(4)));
static volatile uint16_t rx_ring_head = 0;
static volatile uint16_t rx_ring_tail = 0;

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
static uint8_t send_packet(uint8_t type, const uint8_t* payload, uint8_t len) {
    /* Safety: Never exceed protocol length limit or buffer size */
    if (len > COMM_MAX_PAYLOAD) return 1;

    /* Drop plot packets immediately if USB is busy to prevent blocking the main loop.
     * For control packets (ACK, GET response, etc.), wait with a short timeout (~1ms). */
    if (type == RSP_PLOT) {
        if (CDC_IsTxBusy()) return 1;
    } else {
        uint32_t wait_timeout = 10000;
        while (CDC_IsTxBusy()) {
            if (--wait_timeout == 0) return 1;
        }
    }

    tx_buf[0] = COMM_HEADER;
    tx_buf[1] = type;
    tx_buf[2] = len;
    if (len > 0 && payload) {
        memcpy(&tx_buf[3], payload, len);
    }
    /* CRC over type + len + payload */
    tx_buf[3 + len] = compute_crc(&tx_buf[1], 2 + len);

    /* Start transmission */
    return CDC_Transmit_FS(tx_buf, 4 + len);
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
        case PID_ID_ISAT_MEAS: {
            MotorID_Result_t res;
            MotorID_GetResults(&res);
            return res.sat_isat;
        }
        case PID_ID_ALPHA_MEAS: {
            MotorID_Result_t res;
            MotorID_GetResults(&res);
            return res.sat_alpha;
        }
        case PID_ID_DT_MEAS: {
            MotorID_Result_t res;
            MotorID_GetResults(&res);
            return res.identified_deadtime_ns;
        }
        case PID_ID_FREQ_MEAS: {
            MotorID_Result_t res;
            MotorID_GetResults(&res);
            return res.selected_freq_hz;
        }
        case PID_ID_FLUX_MEAS: {
            MotorID_Result_t res;
            MotorID_GetResults(&res);
            return res.measured_flux;
        }
        case PID_ID_KV_MEAS: {
            MotorID_Result_t res;
            MotorID_GetResults(&res);
            return res.measured_kv;
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
        case PID_ID_ISAT_MEAS:
        case PID_ID_ALPHA_MEAS:
        case PID_ID_DT_MEAS:
        case PID_ID_FREQ_MEAS:
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

        case CMD_VOLTAGE: {
            if (len < 4) {
                send_ack(type, 1);
                break;
            }
            float pct;
            memcpy(&pct, payload, 4);
            FOC_SetControlMode(FOC_MODE_VOLTAGE);
            FOC_SetVoltageRef(pct);
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

        case CMD_IDENT_FLUX:
            if (FOC_GetState() != FOC_STATE_IDLE) {
                send_ack(type, 1);
                break;
            }
            MotorID_MeasureFluxOffline();
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

void Comm_PushPlotSample7(float Ia, float Ib, float Ic,
                          float Vd, float Vq, float theta, float Iq_ref) {
    if (!g_foc.plot.enabled) return;

    uint16_t head = plot_ring_head;
    uint16_t next_head = (head + 1) & PLOT_RING_MASK;
    if (next_head == plot_ring_tail) {
        /* Buffer full: drop oldest sample to keep stream real-time */
        plot_ring_tail = (plot_ring_tail + 1) & PLOT_RING_MASK;
    }

    uint16_t offset = head * PLOT_CHANNELS;
    plot_ring_buf[offset + 0] = (int16_t)(Ia * 1000.0f);
    plot_ring_buf[offset + 1] = (int16_t)(Ib * 1000.0f);
    plot_ring_buf[offset + 2] = (int16_t)(Ic * 1000.0f);
    plot_ring_buf[offset + 3] = (int16_t)(Vd * 1000.0f);
    plot_ring_buf[offset + 4] = (int16_t)(Vq * 1000.0f);
    plot_ring_buf[offset + 5] = (int16_t)(theta * 10000.0f);
    plot_ring_buf[offset + 6] = (int16_t)(Iq_ref * 1000.0f);

    plot_ring_head = next_head;
}

void Comm_SendPlotPacket(void) {
    if (!g_foc.plot.enabled) {
        plot_ring_head = 0;
        plot_ring_tail = 0;
        return;
    }

    /* Check if USB is busy first before popping from ring buffer */
    if (CDC_IsTxBusy()) return;

    uint16_t head = plot_ring_head;
    uint16_t tail = plot_ring_tail;
    uint16_t count = (head >= tail) ? (head - tail) : (PLOT_RING_SIZE + head - tail);

    if (count < PLOT_BATCH_SIZE) {
        return; /* Not enough samples for a full packet */
    }

    /* Copy PLOT_BATCH_SIZE samples into packet buffer */
    uint16_t next_tail = tail;
    for (uint8_t i = 0; i < PLOT_BATCH_SIZE; i++) {
        uint16_t src_offset = next_tail * PLOT_CHANNELS;
        uint16_t dst_offset = i * PLOT_CHANNELS;
        for (uint8_t ch = 0; ch < PLOT_CHANNELS; ch++) {
            plot_batch_buf[dst_offset + ch] = plot_ring_buf[src_offset + ch];
        }
        next_tail = (next_tail + 1) & PLOT_RING_MASK;
    }

    if (send_packet(RSP_PLOT, (uint8_t*)plot_batch_buf, PLOT_BATCH_SIZE * PLOT_CHANNELS * 2) == USBD_OK) {
        plot_ring_tail = next_tail;
    }
}
