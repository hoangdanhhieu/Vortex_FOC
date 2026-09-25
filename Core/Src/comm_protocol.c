/**
 * @file comm_protocol.c
 * @brief Binary communication protocol implementation
 *
 * State-machine packet parser for USB CDC binary protocol.
 * Handles parameter SET/GET, motor control, Flash operations,
 * and continuous binary stream telemetry via circular ring buffer.
 */

#include "comm_protocol.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "flash_config.h"
#include "foc_state_machine.h"
#include "motor_id.h"
#include "response_profiler.h"
#include "usbd_cdc_if.h"

static float s_boot_pwm_freq = 0.0f;

/** Parser state machine */
typedef enum {
    PARSE_WAIT_HEADER,
    PARSE_READ_TYPE,
    PARSE_READ_LEN,
    PARSE_READ_PAYLOAD,
    PARSE_READ_CRC
} ParseState_t;
static ParseState_t parse_state = PARSE_WAIT_HEADER;
static uint8_t rx_type;
static uint8_t rx_len;
static uint8_t rx_payload[COMM_MAX_PAYLOAD] __attribute__((aligned(4)));
static uint8_t rx_idx;

static uint8_t tx_buf[COMM_MAX_PAYLOAD + 4]
    __attribute__((aligned(4))); /* header + type + len + payload + crc */

#define RX_RING_SIZE 512
static uint8_t rx_ring_buf[RX_RING_SIZE] __attribute__((aligned(4)));
static volatile uint16_t rx_ring_head = 0;
static volatile uint16_t rx_ring_tail = 0;
static void comm_process_byte(uint8_t byte);

static volatile uint8_t s_param_all_pending = 0;
static uint8_t s_param_all_pid = 0;
static void comm_send_next_param_batch(void);

/*===========================================================================*/
/* Continuous Streaming Ring Buffer                                          */
/*===========================================================================*/

#define STREAM_RING_SIZE 512   /* Must be power-of-2 */
#define STREAM_SETS_PER_PKT 28 /* floor((255-3)/8) = 31, use 28 for clean alignment */

typedef struct {
    __fp16 ch[4]; /* 4 channels x fp16 = 8 bytes/set */
} StreamSet_t;

static StreamSet_t stream_ring[STREAM_RING_SIZE];
static volatile uint16_t s_wr = 0; /* Written by ISR (48kHz) */
static volatile uint16_t s_rd = 0; /* Read by drain (1kHz slow task) */
volatile uint8_t stream_active = 0;

static uint8_t s_channels[4];
static uint8_t s_num_ch = 0;
static uint8_t s_decimation = 1;
static uint8_t s_dec_cnt = 0;
static uint16_t s_seq = 0;

static const float s_dummy_zero = 0.0f;
static const float* s_stream_ptrs[4] = {&s_dummy_zero, &s_dummy_zero, &s_dummy_zero, &s_dummy_zero};

/* Static TX buffer for drain — avoids stack allocation in slow task */
static uint8_t s_drain_payload[3 + STREAM_SETS_PER_PKT * 8];

/*===========================================================================*/
/* Helpers                                                                   */
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
 */
static uint8_t send_packet(uint8_t type, const uint8_t* payload, uint8_t len) {
    if (len > COMM_MAX_PAYLOAD) return 1;

    /* Drop plot/stream packets immediately if USB is busy.
     * For control packets (ACK, GET response, etc.) wait with safe timeout (~5ms @ 170MHz). */
    if (type == RSP_STREAM_DATA) {
        if (CDC_IsTxBusy()) return 1;
    } else {
        uint32_t wait_timeout = 250000;
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
    tx_buf[3 + len] = compute_crc(&tx_buf[1], 2 + len);

    return CDC_Transmit_FS(tx_buf, 4 + len);
}

/*===========================================================================*/
/* Parameter access                                                          */
/*===========================================================================*/

static float get_param_value(uint8_t pid) {
    FlashConfig_t* cfg = FlashConfig_Get();
    switch (pid) {
#define PARAM_DEF(id, type, name, default_val) \
    case id:                                   \
        return (float)cfg->name;
#include "param_table.def"
        case PID_SPD_REF:
            return g_foc.cmd.speed_ref;
        case PID_TRQ_REF:
            return g_foc.cmd.Iq_ref;
        case PID_VBUS:
            return g_foc.data.Vbus;
        case PID_RPM:
            return g_foc.data.omega_elec;
        case PID_ID_MEAS:
            return g_foc.data.Id;
        case PID_IQ_MEAS:
            return g_foc.data.Iq;
        case PID_IA:
            return g_foc.data.Ia;
        case PID_IB:
            return g_foc.data.Ib;
        case PID_IC:
            return g_foc.data.Ic;
        case PID_DUTY_A:
            return g_foc.data.duty_a;
        case PID_DUTY_B:
            return g_foc.data.duty_b;
        case PID_DUTY_C:
            return g_foc.data.duty_c;
        case PID_VD:
            return g_foc.data.Vd;
        case PID_VQ:
            return g_foc.data.Vq;
        case PID_ID_RS_MEAS:
            return id_result.measured_rs;
        case PID_ID_LS_MEAS:
            return id_result.measured_ls;
        case PID_ID_ISAT_MEAS:
            return id_result.sat_isat;
        case PID_ID_ALPHA_MEAS:
            return id_result.sat_alpha;
        case PID_ID_DT_MEAS:
            return id_result.identified_deadtime_ns;
        case PID_ID_FREQ_MEAS:
            return id_result.selected_freq_hz;
        case PID_ID_FLUX_MEAS:
            return id_result.measured_flux;
        case PID_ID_KV_MEAS:
            return id_result.measured_kv;
        case PID_ID_INERTIA_MEAS:
            return id_result.measured_inertia;
        case PID_ID_B0_MEAS:
            return id_result.measured_b0;
        case PID_THETA_ELEC:
            return g_foc.data.theta_elec;
        case PID_USER_PLOT1:
            return g_foc.plot.user_plot1;
        case PID_USER_PLOT2:
            return g_foc.plot.user_plot2;
        case PID_USER_PLOT3:
            return g_foc.plot.user_plot3;
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
        default:
            return 0;
    }
}

/*===========================================================================*/
/* Streaming — ISR fast-path                                                 */
/*===========================================================================*/

/**
 * @brief Resolve a telemetry parameter ID to a direct float pointer.
 *        Called ONCE in background when CMD_STREAM_START is received.
 *        Placed in Flash (runs outside 48kHz ISR).
 */
static const float* stream_resolve_ptr(uint8_t pid) {
    switch (pid) {
        case PID_IA:
            return &g_foc.data.Ia;
        case PID_IB:
            return &g_foc.data.Ib;
        case PID_IC:
            return &g_foc.data.Ic;
        case PID_ID_MEAS:
            return &g_foc.data.Id;
        case PID_IQ_MEAS:
            return &g_foc.data.Iq;
        case PID_RPM:
            return &g_foc.data.omega_elec;
        case PID_SPD_REF:
            return &g_foc.cmd.speed_ref;
        case PID_TRQ_REF:
            return &g_foc.cmd.Iq_ref;
        case PID_VBUS:
            return &g_foc.data.Vbus;
        case PID_VD:
            return &g_foc.data.Vd;
        case PID_VQ:
            return &g_foc.data.Vq;
        case PID_DUTY_A:
            return &g_foc.data.duty_a;
        case PID_DUTY_B:
            return &g_foc.data.duty_b;
        case PID_DUTY_C:
            return &g_foc.data.duty_c;
        case PID_THETA_ELEC:
            return &g_foc.data.theta_elec;
        case PID_USER_PLOT1:
            return &g_foc.plot.user_plot1;
        case PID_USER_PLOT2:
            return &g_foc.plot.user_plot2;
        case PID_USER_PLOT3:
            return &g_foc.plot.user_plot3;
        default:
            return &s_dummy_zero;
    }
}

/**
 * @brief Push one sample-set into the ring buffer.
 *        Called from the HF ISR inside FOC_HighFrequencyTask().
 *        Handles decimation internally. Drops silently on overflow (ring full).
 *        Optimized: Direct pointer dereference, zero cross-memory function calls,
 *        zero branching in channel assignments.
 */
CCMRAM_FUNC void Comm_StreamPush(void) {
    if (!stream_active) return;

    if (++s_dec_cnt < s_decimation) return;
    s_dec_cnt = 0;

    /* Check ring overflow — drop set silently, never block ISR */
    uint16_t next_wr = (s_wr + 1u) & (STREAM_RING_SIZE - 1u);
    if (next_wr == s_rd) return;

    StreamSet_t* set = &stream_ring[s_wr];
    set->ch[0] = (__fp16)(*s_stream_ptrs[0]);
    set->ch[1] = (__fp16)(*s_stream_ptrs[1]);
    set->ch[2] = (__fp16)(*s_stream_ptrs[2]);
    set->ch[3] = (__fp16)(*s_stream_ptrs[3]);

    /* Atomic uint16 store — Cortex-M4 guarantees atomicity for aligned 16-bit */
    s_wr = next_wr;
}

/*===========================================================================*/
/* Streaming — Drain (1kHz slow task)                                        */
/*===========================================================================*/

/**
 * @brief Drain available sets from ring buffer and transmit as RSP_STREAM_DATA.
 *        Packet format: seq(2B) + num_sets(1B) + [ch0..ch3 fp16] x num_sets
 *        Called from FOC_SlowTask() @ 1kHz.
 */
void Comm_StreamDrain(void) {
    if (!stream_active) return;
    if (CDC_IsTxBusy()) return;

    /* Snapshot write pointer (s_wr may advance during drain, that is safe) */
    uint16_t wr = s_wr;
    uint16_t avail = (wr - s_rd) & (STREAM_RING_SIZE - 1u);
    if (avail == 0u) return;

    uint8_t num_sets = (avail > STREAM_SETS_PER_PKT) ? STREAM_SETS_PER_PKT : (uint8_t)avail;

    /* Build payload: [seq_lo][seq_hi][num_sets][set0_ch0..ch3][set1...]... */
    s_drain_payload[0] = (uint8_t)(s_seq & 0xFFu);
    s_drain_payload[1] = (uint8_t)(s_seq >> 8u);
    s_drain_payload[2] = num_sets;
    s_seq++;

    uint8_t* p = &s_drain_payload[3];
    for (uint8_t i = 0; i < num_sets; i++) {
        StreamSet_t* set = &stream_ring[(s_rd + i) & (STREAM_RING_SIZE - 1u)];
        memcpy(p, set->ch, 8u);
        p += 8u;
    }

    uint8_t plen = 3u + num_sets * 8u;
    if (send_packet(RSP_STREAM_DATA, s_drain_payload, plen) == 0) {
        /* Advance read pointer only on successful transmit */
        s_rd = (s_rd + num_sets) & (STREAM_RING_SIZE - 1u);
    }
}

/*===========================================================================*/
/* Packet handler                                                            */
/*===========================================================================*/

static void send_ack(uint8_t cmd_type, uint8_t status) {
    uint8_t payload[2] = {cmd_type, status};
    send_packet(RSP_ACK, payload, 2);
}

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

            /* Prohibit changing PWM frequency while motor is operating */
            if (pid == PID_PWM_FREQ) {
                if (FOC_GetState() != FOC_STATE_IDLE && FOC_GetState() != FOC_STATE_STOP &&
                    FOC_GetState() != FOC_STATE_FAULT) {
                    send_ack(type, 1);
                    break;
                }
            }

            float val;
            memcpy(&val, &payload[1], 4);
            set_param_value(pid, val);

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

        case CMD_SAVE: {
            bool freq_changed = (fabsf(cfg->pwm_frequency - s_boot_pwm_freq) > 1.0f);
            int res = FlashConfig_Save();
            send_ack(type, (res == 0) ? 0 : 1);
            if (res == 0 && freq_changed) {
                for (volatile uint32_t i = 0; i < 2000000; i++) {
                    __NOP();
                }
                NVIC_SystemReset();
            }
            break;
        }

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
            float speed_rad;
            memcpy(&speed_rad, payload, 4);
            FOC_SetControlMode(FOC_MODE_SPEED);
            FOC_SetSpeedRef(speed_rad);
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

        case CMD_STATUS: {
            uint8_t rsp[16];
            rsp[0] = (uint8_t)g_foc.status.state;
            rsp[1] = (uint8_t)g_foc.status.fault;
            rsp[2] = (FOC_GetDirection() < 0) ? 1 : 0;
            rsp[3] = 0;
            memcpy(&rsp[4], &g_foc.data.omega_elec, 4);
            memcpy(&rsp[8], &g_foc.data.Vbus, 4);
            memcpy(&rsp[12], &g_foc.data.Ibus, 4);
            send_packet(RSP_STATUS, rsp, 16);
            break;
        }

        case CMD_PARAM_ALL: {
            s_param_all_pending = 1;
            s_param_all_pid = 0;
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

        case CMD_IDENT_INERTIA:
            if (FOC_GetState() != FOC_STATE_IDLE) {
                send_ack(type, 1);
                break;
            }
            MotorID_MeasureInertiaOffline();
            send_ack(type, 0);
            break;

        case CMD_CLEAR:
            FOC_ClearFault();
            send_ack(type, 0);
            break;

        case CMD_PROFILER: {
            if (len >= 13) {
                uint8_t mode = payload[0];
                float amp_args, offset_args, freq_args;
                memcpy(&amp_args, &payload[1], 4);
                memcpy(&offset_args, &payload[5], 4);
                memcpy(&freq_args, &payload[9], 4);
                Profiler_Start(&g_foc.ctrl.profiler, mode, amp_args, offset_args, freq_args);
                if (mode != PROFILER_MODE_OFF) {
                    FOC_SetControlMode(FOC_MODE_TORQUE);
                }
                send_ack(type, 0);
            } else {
                send_ack(type, 1);
            }
            break;
        }

        /* ── Continuous Streaming ── */
        case CMD_STREAM_START: {
            if (len < 2) {
                send_ack(type, 1);
                break;
            }
            uint8_t num_ch = payload[0];
            if (num_ch == 0u || num_ch > 4u) {
                send_ack(type, 1);
                break;
            }
            uint8_t dec = payload[1];
            if (dec < STREAM_DEC_MIN) dec = STREAM_DEC_MIN;
            if (dec > STREAM_DEC_MAX) dec = STREAM_DEC_MAX;

            /* Stop any active stream before reconfiguring */
            stream_active = 0;

            s_num_ch = num_ch;
            s_decimation = dec;
            for (uint8_t i = 0; i < 4u; i++) {
                if (i < s_num_ch) {
                    uint8_t pid = (2u + i < len) ? payload[2u + i] : 0u;
                    s_channels[i] = pid;
                    s_stream_ptrs[i] = stream_resolve_ptr(pid);
                } else {
                    s_channels[i] = 0u;
                    s_stream_ptrs[i] = &s_dummy_zero;
                }
            }

            /* Reset ring buffer and counters */
            s_wr = 0;
            s_rd = 0;
            s_dec_cnt = 0;
            s_seq = 0;

            stream_active = 1;
            send_ack(type, 0);
            break;
        }

        case CMD_STREAM_STOP:
            stream_active = 0;
            send_ack(type, 0);
            break;

        default:
            send_ack(type, 1);
            break;
    }
}

/*===========================================================================*/
/* Param-all batch sender                                                    */
/*===========================================================================*/

static void comm_send_next_param_batch(void) {
    if (CDC_IsTxBusy()) return;

    uint8_t rsp[1 + 32 * 5];
    uint8_t count = 0;
    uint8_t pos = 1;
    uint8_t start_pid = s_param_all_pid;

    while (s_param_all_pid < PID_COUNT && count < 32) {
        uint8_t pid = s_param_all_pid++;
        if (!is_pid_defined(pid)) continue;

        rsp[pos++] = pid;
        float val = get_param_value(pid);
        memcpy(&rsp[pos], &val, 4);
        pos += 4;
        count++;
    }

    if (count > 0) {
        rsp[0] = count;
        if (send_packet(RSP_PARAM_ALL, rsp, pos) != 0) {
            /* Rollback PID if USB transfer failed or busy */
            s_param_all_pid = start_pid;
            return;
        }
    }

    if (s_param_all_pid >= PID_COUNT) {
        s_param_all_pending = 0;
        s_param_all_pid = 0;
    }
}

/*===========================================================================*/
/* Public API                                                                */
/*===========================================================================*/

void Comm_Init(void) {
    parse_state = PARSE_WAIT_HEADER;
    rx_idx = 0;
    rx_ring_head = 0;
    rx_ring_tail = 0;
    s_param_all_pending = 0;
    s_param_all_pid = 0;
    stream_active = 0;
    s_wr = 0;
    s_rd = 0;
    s_seq = 0;
    s_dec_cnt = 0;
    s_boot_pwm_freq = FlashConfig_Get()->pwm_frequency;
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

    if (s_param_all_pending) {
        comm_send_next_param_batch();
    } else {
        Comm_StreamDrain();
    }
}

void Comm_SendStatus(void) {
    uint8_t rsp[16];
    rsp[0] = (uint8_t)g_foc.status.state;
    rsp[1] = (uint8_t)g_foc.status.fault;
    rsp[2] = (FOC_GetDirection() < 0) ? 1 : 0;
    rsp[3] = 0;
    memcpy(&rsp[4], &g_foc.data.omega_elec, 4);
    memcpy(&rsp[8], &g_foc.data.Vbus, 4);
    memcpy(&rsp[12], &g_foc.data.Ibus, 4);
    send_packet(RSP_STATUS, rsp, 16);
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
                parse_state = PARSE_WAIT_HEADER;
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
            uint8_t crc = rx_type ^ rx_len;
            for (uint8_t i = 0; i < rx_len; i++) {
                crc ^= rx_payload[i];
            }
            if (crc == byte) {
                handle_packet(rx_type, rx_payload, rx_len);
            }
            parse_state = PARSE_WAIT_HEADER;
            break;
        }
    }
}
