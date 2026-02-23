/**
 * @file uart_cmd.c
 * @brief Command Parser Implementation — USB CDC backend
 *
 * All debug/command I/O now goes through USB CDC (Virtual COM Port).
 * Unified parameter table for runtime tuning + persistent Flash storage.
 */

#include "uart_cmd.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flash_config.h"
#include "foc_state_machine.h"
#include "usbd_cdc_if.h"

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

static char cmd_buffer[UART_CMD_BUFFER_SIZE];
static uint8_t cmd_index = 0;
static uint8_t plot_enabled = 0;
static char tx_buffer[UART_TX_BUFFER_SIZE];

/*===========================================================================*/
/* Parameter Table                                                           */
/*===========================================================================*/

/** Parameter flags */
#define P_CFG 0x00  /**< Config param: saved to Flash */
#define P_LIVE 0x01 /**< Live param: realtime, not saved */
#define P_RO 0x02   /**< Read-only (combine with P_LIVE) */

typedef struct {
    const char* name;
    float* ptr;
    uint8_t flags;
} ParamEntry_t;

/**
 * Parameter table — unified list of all tunable/readable parameters.
 * To add a new parameter: add 1 field to FlashConfig_t + 1 line here.
 */
static ParamEntry_t param_table[50] = {0};

#define PARAM_COUNT (sizeof(param_table) / sizeof(param_table[0]))

/**
 * @brief Initialize the parameter table pointers
 *        Called once after FlashConfig_Init()
 */
static void init_param_table(void) {
    FlashConfig_t* cfg = FlashConfig_Get();
    uint16_t i = 0;

    /* Current PI */
    param_table[i++] = (ParamEntry_t){"KP_ID", &cfg->kp_id, P_CFG};
    param_table[i++] = (ParamEntry_t){"KI_ID", &cfg->ki_id, P_CFG};
    param_table[i++] = (ParamEntry_t){"KP_IQ", &cfg->kp_iq, P_CFG};
    param_table[i++] = (ParamEntry_t){"KI_IQ", &cfg->ki_iq, P_CFG};
    param_table[i++] = (ParamEntry_t){"BW_CUR", &cfg->bw_cur, P_CFG};

    /* Speed PI */
    param_table[i++] = (ParamEntry_t){"KP_SPD", &cfg->kp_spd, P_CFG};
    param_table[i++] = (ParamEntry_t){"KI_SPD", &cfg->ki_spd, P_CFG};
    param_table[i++] = (ParamEntry_t){"BW_SPD", &cfg->bw_spd, P_CFG};

    /* Startup */
    param_table[i++] = (ParamEntry_t){"I_STRT", &cfg->startup_current, P_CFG};
    param_table[i++] = (ParamEntry_t){"I_ALGN", &cfg->align_current, P_CFG};
    param_table[i++] = (ParamEntry_t){"ACCEL", &cfg->startup_accel, P_CFG};
    param_table[i++] = (ParamEntry_t){"HANDOFF", &cfg->startup_handoff_speed, P_CFG};

    /* Ramp Rates */
    param_table[i++] = (ParamEntry_t){"RAMP_ACC", &cfg->speed_ramp_accel, P_CFG};
    param_table[i++] = (ParamEntry_t){"RAMP_DEC", &cfg->speed_ramp_decel, P_CFG};
    param_table[i++] = (ParamEntry_t){"I_RAMP", &cfg->current_ramp_rate, P_CFG};

    /* Motor & SMO */
    param_table[i++] = (ParamEntry_t){"M_RS", &cfg->motor_rs, P_CFG};
    param_table[i++] = (ParamEntry_t){"M_LS", &cfg->motor_ls, P_CFG};
    param_table[i++] = (ParamEntry_t){"M_KV", &cfg->motor_kv, P_CFG};
    param_table[i++] = (ParamEntry_t){"M_FLUX", &cfg->motor_flux, P_CFG};
    param_table[i++] = (ParamEntry_t){"M_POLES", &cfg->motor_poles, P_CFG};
    param_table[i++] = (ParamEntry_t){"M_MAX_SP", &cfg->motor_max_spd, P_CFG};
    param_table[i++] = (ParamEntry_t){"M_MIN_SP", &cfg->motor_min_spd, P_CFG};
    param_table[i++] = (ParamEntry_t){"M_MAX_I", &cfg->motor_max_curr, P_CFG};
    param_table[i++] = (ParamEntry_t){"M_NOM_V", &cfg->motor_nom_vbus, P_CFG};
    param_table[i++] = (ParamEntry_t){"SMO_KS", &cfg->smo_k_slide, P_CFG};
    param_table[i++] = (ParamEntry_t){"SMO_SIG", &cfg->smo_k_sigmoid, P_CFG};
    param_table[i++] = (ParamEntry_t){"SMO_BEMF", &cfg->smo_bemf_cutoff, P_CFG};
    param_table[i++] = (ParamEntry_t){"SMO_PLL", &cfg->smo_pll_cutoff, P_CFG};
    param_table[i++] = (ParamEntry_t){"COMP_DELAY", (float*)&cfg->comp_delay_samples, P_CFG};

    /* ADC */
    param_table[i++] = (ParamEntry_t){"ADC_MARG", (float*)&cfg->adc_margin_ticks, P_CFG};

    /* Safety */
    param_table[i++] = (ParamEntry_t){"OC_THR", &cfg->fault_oc_threshold, P_CFG};
    param_table[i++] = (ParamEntry_t){"OV_THR", &cfg->fault_ov_threshold, P_CFG};
    param_table[i++] = (ParamEntry_t){"UV_THR", &cfg->fault_uv_threshold, P_CFG};
    param_table[i++] = (ParamEntry_t){"STALL_SPD", &cfg->fault_stall_speed, P_CFG};
    param_table[i++] = (ParamEntry_t){"STALL_I", &cfg->fault_stall_current, P_CFG};
    param_table[i++] = (ParamEntry_t){"STALL_MS", (float*)&cfg->fault_stall_time_ms, P_CFG};

    /* Live Params (realtime, not saved) */
    param_table[i++] = (ParamEntry_t){"SPD", &g_foc.cmd.speed_ref_target, P_LIVE};
    param_table[i++] = (ParamEntry_t){"TRQ", &g_foc.cmd.Iq_ref_target, P_LIVE};
    param_table[i++] = (ParamEntry_t){"VBUS", &g_foc.data.Vbus, P_LIVE | P_RO};
    param_table[i++] = (ParamEntry_t){"RPM", &g_foc.data.speed_rpm, P_LIVE | P_RO};
    param_table[i++] = (ParamEntry_t){"ID", &g_foc.data.Id, P_LIVE | P_RO};
    param_table[i++] = (ParamEntry_t){"IQ", &g_foc.data.Iq, P_LIVE | P_RO};
    param_table[i++] = (ParamEntry_t){"IA", &g_foc.data.Ia, P_LIVE | P_RO};
    param_table[i++] = (ParamEntry_t){"IB", &g_foc.data.Ib, P_LIVE | P_RO};
}

/*===========================================================================*/
/* Private Functions                                                         */
/*===========================================================================*/

static void UART_CMD_Execute(const char* cmd);
static void UART_CMD_ParseCommand(const char* cmd, char* cmd_name, char* arg_str, float* arg);

/*===========================================================================*/
/* USB CDC Helper                                                            */
/*===========================================================================*/

/**
 * @brief Send data via USB CDC with busy-wait retry
 * @param data Pointer to data buffer
 * @param len  Number of bytes to send
 */
static void USB_SendData(const uint8_t* data, uint16_t len) {
    if (len == 0) return;

    /* Retry a limited number of times if the TX endpoint is busy */
    uint32_t retries = 100000;
    while (CDC_Transmit_FS((uint8_t*)data, len) == USBD_BUSY) {
        if (--retries == 0) break; /* give up to avoid blocking forever */
    }
}

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

void UART_CMD_Init(void) {
    cmd_index = 0;
    plot_enabled = 0;
    memset(cmd_buffer, 0, sizeof(cmd_buffer));

    /* Initialize parameter table with live pointers */
    init_param_table();
}

void UART_CMD_ProcessByte(uint8_t data) {
    if (data == '\n' || data == '\r') {
        if (cmd_index > 0) {
            cmd_buffer[cmd_index] = '\0';
            UART_CMD_Execute(cmd_buffer);
            cmd_index = 0;
        }
    } else if (cmd_index < UART_CMD_BUFFER_SIZE - 1) {
        cmd_buffer[cmd_index++] = (char)data;
    }
}

void UART_CMD_SendString(const char* str) {
    USB_SendData((const uint8_t*)str, strlen(str));
}

void UART_CMD_Printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int len = vsnprintf(tx_buffer, UART_TX_BUFFER_SIZE, format, args);
    va_end(args);

    if (len > 0) {
        USB_SendData((const uint8_t*)tx_buffer, len);
    }
}

void UART_CMD_SendPlotData(void) {
    if (!plot_enabled) return;

    /* Format: Ia,Ib,Id,Iq,theta,speed_rpm\n */
    int len = snprintf(tx_buffer, UART_TX_BUFFER_SIZE, "%.2f,%.2f,%.2f,%.2f,%.3f,%.0f\n",
                       g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Id, g_foc.data.Iq,
                       g_foc.data.theta_elec, g_foc.data.speed_rpm);

    if (len > 0) {
        USB_SendData((const uint8_t*)tx_buffer, len);
    }
}

void UART_CMD_EnablePlot(uint8_t enable) {
    plot_enabled = enable;
}

uint8_t UART_CMD_IsPlotEnabled(void) {
    return plot_enabled;
}

/*===========================================================================*/
/* Command Parsing                                                           */
/*===========================================================================*/

/**
 * @brief Parse command into name, raw argument string, and float value
 */
static void UART_CMD_ParseCommand(const char* cmd, char* cmd_name, char* arg_str, float* arg) {
    *arg = 0.0f;
    arg_str[0] = '\0';

    /* Find first space separator */
    const char* space = strchr(cmd, ' ');
    if (space) {
        int name_len = space - cmd;
        if (name_len > 15) name_len = 15;
        strncpy(cmd_name, cmd, name_len);
        cmd_name[name_len] = '\0';

        /* Copy raw argument string (for compound args like "SET KP_IQ 0.5") */
        strncpy(arg_str, space + 1, 63);
        arg_str[63] = '\0';

        /* Parse first numeric argument */
        *arg = atof(space + 1);
    } else {
        strncpy(cmd_name, cmd, 16);
        cmd_name[15] = '\0';
    }

    /* Convert command name to uppercase */
    for (int i = 0; cmd_name[i]; i++) {
        if (cmd_name[i] >= 'a' && cmd_name[i] <= 'z') {
            cmd_name[i] -= 32;
        }
    }
}

/*===========================================================================*/
/* Parameter Table Lookup                                                    */
/*===========================================================================*/

/**
 * @brief Find a parameter by name
 * @return Pointer to entry, or NULL if not found
 */
static ParamEntry_t* find_param(const char* name) {
    /* Uppercase the name for comparison */
    char upper[16];
    int i;
    for (i = 0; name[i] && i < 15; i++) {
        upper[i] = (name[i] >= 'a' && name[i] <= 'z') ? name[i] - 32 : name[i];
    }
    upper[i] = '\0';

    for (uint32_t j = 0; j < PARAM_COUNT; j++) {
        if (param_table[j].name && strcmp(param_table[j].name, upper) == 0) {
            return &param_table[j];
        }
    }
    return NULL;
}

/*===========================================================================*/
/* Command Execution                                                         */
/*===========================================================================*/

static void UART_CMD_Execute(const char* cmd) {
    char cmd_name[16];
    char arg_str[64];
    float arg;

    UART_CMD_ParseCommand(cmd, cmd_name, arg_str, &arg);

    /*--- Basic commands (shortcuts) ---*/
    if (strcmp(cmd_name, "PING") == 0) {
        UART_CMD_SendString("PONG\r\n");

    } else if (strcmp(cmd_name, "START") == 0) {
        FOC_Start();
        UART_CMD_SendString("OK: Motor starting\r\n");

    } else if (strcmp(cmd_name, "STOP") == 0) {
        FOC_Stop();
        UART_CMD_SendString("OK: Motor stopping\r\n");

    } else if (strcmp(cmd_name, "SPD") == 0) {
        FOC_SetControlMode(FOC_MODE_SPEED);
        FOC_SetSpeedRef(arg);
        UART_CMD_Printf("OK: Speed ref = %.0f RPM\r\n", arg);

    } else if (strcmp(cmd_name, "TRQ") == 0) {
        FOC_SetControlMode(FOC_MODE_TORQUE);
        FOC_SetTorqueRef(arg);
        UART_CMD_Printf("OK: Torque ref = %.0f%s\r\n", arg, "%");

    } else if (strcmp(cmd_name, "DIR") == 0) {
        if (FOC_GetState() != FOC_STATE_IDLE) {
            UART_CMD_SendString("ERR: Stop motor first\r\n");
        } else {
            int8_t dir = (arg < 0) ? -1 : 1;
            FOC_SetDirection(dir);
            UART_CMD_Printf("OK: Direction = %s\r\n", (dir < 0) ? "REV" : "FWD");
        }

    } else if (strcmp(cmd_name, "STATUS") == 0) {
        const char* state_names[] = {"IDLE", "CAL",  "ALIGN", "STARTUP",
                                     "RUN",  "STOP", "FAULT", "IDENT"};
        UART_CMD_Printf("State: %s, Speed: %.0f RPM, Vbus: %.1f V, Dir: %s\r\n",
                        state_names[FOC_GetState()], g_foc.data.speed_rpm, g_foc.data.Vbus,
                        (FOC_GetDirection() < 0) ? "REV" : "FWD");

    } else if (strcmp(cmd_name, "CLEAR") == 0) {
        FOC_ClearFault();
        UART_CMD_SendString("OK: Fault cleared\r\n");

        /*--- Plot control ---*/
    } else if (strcmp(cmd_name, "PLOT") == 0) {
        uint8_t en = (arg > 0.5f) ? 1 : 0;
        UART_CMD_EnablePlot(en);
        UART_CMD_Printf("OK: Plot %s\r\n", en ? "ON" : "OFF");

        /*--- Motor identification ---*/
    } else if (strcmp(cmd_name, "IDENT") == 0) {
        if (FOC_GetState() != FOC_STATE_IDLE) {
            UART_CMD_SendString("ERR: Stop motor first\r\n");
        } else {
            FOC_StartSelfCommission();
            UART_CMD_SendString("OK: Self-commissioning started\r\n");
        }

        /*--- SET <param> <value> ---*/
    } else if (strcmp(cmd_name, "SET") == 0) {
        /* Parse "PARAM_NAME VALUE" from arg_str */
        char param_name[16];
        float param_val = 0.0f;
        const char* sp = strchr(arg_str, ' ');
        if (!sp) {
            UART_CMD_SendString("ERR: Usage: SET <name> <value>\r\n");
        } else {
            int plen = sp - arg_str;
            if (plen > 15) plen = 15;
            strncpy(param_name, arg_str, plen);
            param_name[plen] = '\0';
            param_val = atof(sp + 1);

            ParamEntry_t* p = find_param(param_name);
            if (!p) {
                UART_CMD_Printf("ERR: Unknown param '%s'\r\n", param_name);
            } else if (p->flags & P_RO) {
                UART_CMD_Printf("ERR: '%s' is read-only\r\n", p->name);
            } else {
                *p->ptr = param_val;

                /* If config param, apply to live system */
                if (!(p->flags & P_LIVE)) {
                    FlashConfig_Apply();
                }

                UART_CMD_Printf("OK: %s = %.6g\r\n", p->name, *p->ptr);
            }
        }

        /*--- GET <param> ---*/
    } else if (strcmp(cmd_name, "GET") == 0) {
        ParamEntry_t* p = find_param(arg_str);
        if (!p) {
            UART_CMD_Printf("ERR: Unknown param '%s'\r\n", arg_str);
        } else {
            UART_CMD_Printf("%s = %.6g\r\n", p->name, *p->ptr);
        }

        /*--- PARAM: list all parameters ---*/
    } else if (strcmp(cmd_name, "PARAM") == 0) {
        UART_CMD_SendString("--- Config (saved) ---\r\n");
        for (uint32_t i = 0; i < PARAM_COUNT; i++) {
            if (!param_table[i].name) continue;
            if (param_table[i].flags & P_LIVE) continue;
            UART_CMD_Printf("  %-10s = %.6g\r\n", param_table[i].name, *param_table[i].ptr);
        }
        UART_CMD_SendString("--- Live (realtime) ---\r\n");
        for (uint32_t i = 0; i < PARAM_COUNT; i++) {
            if (!param_table[i].name) continue;
            if (!(param_table[i].flags & P_LIVE)) continue;
            UART_CMD_Printf("  %-10s = %.6g %s\r\n", param_table[i].name, *param_table[i].ptr,
                            (param_table[i].flags & P_RO) ? "(RO)" : "");
        }

        /*--- SAVE: write config to Flash ---*/
    } else if (strcmp(cmd_name, "SAVE") == 0) {
        if (FlashConfig_Save() == 0) {
            UART_CMD_SendString("OK: Config saved to Flash\r\n");
        } else {
            UART_CMD_SendString("ERR: Flash write failed\r\n");
        }

        /*--- LOAD: load config from Flash ---*/
    } else if (strcmp(cmd_name, "LOAD") == 0) {
        FlashConfig_Init();
        FlashConfig_Apply();
        init_param_table();
        UART_CMD_SendString("OK: Config loaded from Flash\r\n");

        /*--- DEFAULTS: reset to compile-time defaults ---*/
    } else if (strcmp(cmd_name, "DEFAULTS") == 0) {
        FlashConfig_LoadDefaults();
        FlashConfig_Apply();
        UART_CMD_SendString("OK: Defaults restored (not saved — use SAVE to persist)\r\n");

        /*--- HELP ---*/
    } else if (strcmp(cmd_name, "HELP") == 0) {
        UART_CMD_SendString(
            "Commands:\r\n"
            "  PING             - Connection test\r\n"
            "  START / STOP     - Motor control\r\n"
            "  SPD <rpm>        - Set speed\r\n"
            "  TRQ <%>          - Set torque\r\n"
            "  DIR <1/-1>       - Set direction\r\n"
            "  STATUS           - Show status\r\n"
            "  CLEAR            - Clear fault\r\n"
            "  PLOT <1/0>       - Plot streaming\r\n"
            "  IDENT            - Self-commission\r\n"
            "  SET <name> <val> - Set parameter\r\n"
            "  GET <name>       - Get parameter\r\n"
            "  PARAM            - List all params\r\n"
            "  SAVE             - Save config to Flash\r\n"
            "  LOAD             - Load config from Flash\r\n"
            "  DEFAULTS         - Reset to defaults\r\n");
    } else {
        UART_CMD_Printf("ERR: Unknown cmd '%s'\r\n", cmd);
    }
}
