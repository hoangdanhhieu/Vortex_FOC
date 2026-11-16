/**
 * @file foc_state_machine.c
 * @brief FOC Core State Machine, High Frequency Task and APIs
 */

#include "foc_state_machine.h"

#include <math.h>
#include <string.h>

#include "comm_protocol.h"
#include "cordic_math.h"
#include "foc.h"
#include "foc_beep.h"
#include "foc_calibration.h"
#include "foc_config.h"
#include "foc_flying_start.h"
#include "foc_hardware.h"
#include "foc_input.h"
#include "foc_startup.h"
#include "main.h"
#include "motor_id.h"
#include "peripheral_init.h"

/*===========================================================================*/
/* Global Instance                                                           */
/*===========================================================================*/

FOC_Control_t g_foc __attribute__((aligned(4)));
static volatile uint8_t foc_initialized = 0;

/**
 * @brief Tracks which phase each ADC was configured to sample.
 *        Set at END of each ISR for the NEXT trigger.
 *        0 = Phase A, 1 = Phase B, 2 = Phase C
 */
static uint8_t s_skip_phase = 2; /* Which phase is currently skipped (0=A, 1=B, 2=C) */
static uint8_t s_cal_phase = 2;  /* Phase skip index during calibration */

extern volatile uint32_t adc_isr_us;
extern volatile uint16_t adc_regular_buffer[3];
extern volatile float ADC_Vref;
static float s_valpha_active = 0.0f;
static float s_vbeta_active = 0.0f;
static void FOC_StateRun(void);
static void FOC_StateZeroCross(void);
NOINLINE_FUNC static void FOC_ZeroCross_Init(float target_dir, float omega_cross);
NOINLINE_FUNC static void FOC_StateSelfCommission(void);
void FOC_StateBeep(void);

uint8_t FOC_IsInitialized(void) {
    return foc_initialized;
}

uint8_t FOC_IsConfigValid(void) {
    if (!isfinite(g_foc.cfg.motor_rs) || g_foc.cfg.motor_rs < 0.001f || g_foc.cfg.motor_rs > 100.0f)
        return 0;
    if (!isfinite(g_foc.cfg.motor_ls) || g_foc.cfg.motor_ls < 0.1e-6f || g_foc.cfg.motor_ls > 0.1f)
        return 0;
    if (!isfinite(g_foc.cfg.motor_poles) || g_foc.cfg.motor_poles < 1.0f ||
        g_foc.cfg.motor_poles > 60.0f)
        return 0;
    if (!isfinite(g_foc.cfg.motor_flux) || g_foc.cfg.motor_flux < 0.00001f ||
        g_foc.cfg.motor_flux > 0.5f)
        return 0;
    if (!isfinite(g_foc.cfg.motor_max_curr) || g_foc.cfg.motor_max_curr < 0.5f ||
        g_foc.cfg.motor_max_curr > 1000.0f)
        return 0;
    if (!isfinite(g_foc.cfg.motor_max_spd) || g_foc.cfg.motor_max_spd < 50.0f ||
        g_foc.cfg.motor_max_spd > 200000.0f)
        return 0;

    if (!isfinite(g_foc.cfg.kp_iq) || g_foc.cfg.kp_iq <= 1e-6f) return 0;
    if (!isfinite(g_foc.cfg.ki_iq) || g_foc.cfg.ki_iq <= 1e-6f) return 0;
    if (!isfinite(g_foc.cfg.kp_id) || g_foc.cfg.kp_id <= 1e-6f) return 0;
    if (!isfinite(g_foc.cfg.ki_id) || g_foc.cfg.ki_id <= 1e-6f) return 0;

    if (!isfinite(g_foc.cfg.fault_ov_threshold) || !isfinite(g_foc.cfg.fault_uv_threshold))
        return 0;
    if (g_foc.cfg.fault_ov_threshold <= g_foc.cfg.fault_uv_threshold) return 0;
    if (g_foc.cfg.deadtime_ns < 50.0f || g_foc.cfg.deadtime_ns > 3000.0f) return 0;
    return 1;
}

void FOC_Init(void) {
    foc_initialized = 0;

    g_foc.status.state = FOC_STATE_IDLE;
    g_foc.status.control_mode = FOC_MODE_SPEED;
    g_foc.status.fault = FOC_FAULT_NONE;

    g_foc.data.Ia = g_foc.data.Ib = g_foc.data.Ic = 0.0f;
    g_foc.data.Ialpha = g_foc.data.Ibeta = 0.0f;
    g_foc.data.Ialpha_flt = g_foc.data.Ibeta_flt = 0.0f;
    g_foc.data.Id = g_foc.data.Iq = 0.0f;
    g_foc.data.Vd = g_foc.data.Vq = 0.0f;
    g_foc.data.Valpha = g_foc.data.Vbeta = 0.0f;
    g_foc.data.Valpha_applied = g_foc.data.Vbeta_applied = 0.0f;
    s_valpha_active = s_vbeta_active = 0.0f;
    g_foc.data.Vbus = 12.0f;
    g_foc.data.Vbus_inv = 1.0f / 12.0f;
    g_foc.data.Ibus = 0.0f;
    g_foc.data.inv_i_th = 2.5f;

    g_foc.cmd.Id_ref = 0.0f;
    g_foc.cmd.Iq_ref = 0.0f;
    g_foc.cmd.Vq_ref = 0.0f;
    g_foc.cmd.Vq_ref_target = 0.0f;
    g_foc.cmd.speed_ref = 0.0f;
    g_foc.data.theta_elec = 0.0f;
    g_foc.data.omega_elec = 0.0f;
    g_foc.status.in_transition = 0;

    g_foc.startup.theta = 0.0f;
    g_foc.startup.omega = 0.0f;
    g_foc.startup.counter = 0;
    g_foc.status.reverse = 1.0f;
    memset(&g_foc.zc, 0, sizeof(g_foc.zc));

    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    g_foc.dt = (g_foc.cfg.pwm_frequency > 0.0f) ? (1.0f / g_foc.cfg.pwm_frequency) : CONTROL_PERIOD;

    PI_Init(&g_foc.ctrl.id, 0.0f, 0.0f, -0.0f, 0.0f, g_foc.dt);
    PI_SetIntLimits(&g_foc.ctrl.id, -12.0f, 12.0f);

    PI_Init(&g_foc.ctrl.iq, 0.0f, 0.0f, -0.0f, 0.0f, g_foc.dt);
    PI_SetIntLimits(&g_foc.ctrl.iq, -0.0f, 0.0f);

    LADRC_Init(&g_foc.ctrl.speed, 0.0f, 0.0f, 1.0f, SPEED_LOOP_OUT_MIN, SPEED_LOOP_OUT_MAX, 0.001f);

    SMO_Init(&g_foc.ctrl.smo);

    Profiler_Init(&g_foc.ctrl.profiler);

    g_foc.status.run_counter = 0;
    g_foc.isr_time_cycles = 0;
    g_foc.data.i_scale = ADC_Vref * (ADC_TO_CURRENT);
    FOC_EnableDrivers(0);

    MotorID_Init();

    foc_initialized = 1;
    FOC_ResetStallDetector();
    FOC_Input_Init();
}

void FOC_Start(void) {
    if (!FOC_IsConfigValid() && !MotorID_IsRunning()) {
        return;
    }
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_BEEP) {
        if (g_foc.status.state == FOC_STATE_BEEP) {
            FOC_Beep_Stop();
            FOC_EnableDrivers(0);
            g_foc.data.duty_a = 0.5f;
            g_foc.data.duty_b = 0.5f;
            g_foc.data.duty_c = 0.5f;
        }
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        LADRC_Reset(&g_foc.ctrl.speed);
        SMO_Reset(&g_foc.ctrl.smo);

        /* Ensure PI controllers use original default gains from config */
        PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
        PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);

        FOC_Startup_Reset();
        g_foc.cmd.Iq_ref = 0.0f;

        FOC_FlyingStart_Init();
        FOC_Calibration_Reset();
        float init_dir = 1.0f;
        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            if (g_foc.cmd.speed_ref_target < -1e-5f) init_dir = -1.0f;
        } else if (g_foc.status.control_mode == FOC_MODE_TORQUE) {
            if (g_foc.cmd.Iq_ref_target < -1e-5f) init_dir = -1.0f;
        } else if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            if (g_foc.cmd.Vq_ref_target < -1e-5f) init_dir = -1.0f;
        }
        g_foc.status.reverse = init_dir;

        g_foc.zc.debounce_cnt = 0;
        g_foc.zc.lockout_cnt = 0;
        g_foc.zc.grace_counter = 0;
        g_foc.zc.was_reversing = 0;

        /* Reset ADC channel switching to default (skip C) */
        s_skip_phase = 2;
        s_cal_phase = 2;
        FOC_HW_SwitchCurrentSensing(s_skip_phase);

        g_foc.status.run_counter = 0;
        g_foc.status.state = FOC_STATE_CALIBRATION;

        float v_limit = g_foc.data.Vbus * SQRT3_INV;
        PI_SetLimits(&g_foc.ctrl.id, -v_limit, v_limit);
        PI_SetIntLimits(&g_foc.ctrl.id, -v_limit, v_limit);
        PI_SetLimits(&g_foc.ctrl.iq, -v_limit, v_limit);
        PI_SetIntLimits(&g_foc.ctrl.iq, -v_limit, v_limit);
    }
}

void FOC_Stop(void) {
    FOC_EnableDrivers(0);
    FOC_Beep_Stop();
    MotorID_Stop();
    g_foc.zc.was_reversing = 0;
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
    g_foc.data.Valpha = g_foc.data.Vbeta = 0.0f;
    g_foc.data.Valpha_applied = g_foc.data.Vbeta_applied = 0.0f;
    s_valpha_active = s_vbeta_active = 0.0f;

    g_foc.cmd.Id_ref = 0.0f;
    g_foc.cmd.Iq_ref = 0.0f;
    g_foc.cmd.Iq_ref_target = 0.0f;
    g_foc.cmd.Vq_ref = 0.0f;
    g_foc.cmd.Vq_ref_target = 0.0f;
    g_foc.cmd.speed_ref = 0.0f;
    g_foc.cmd.speed_ref_target = 0.0f;

    PI_Reset(&g_foc.ctrl.id);
    PI_Reset(&g_foc.ctrl.iq);
    LADRC_Reset(&g_foc.ctrl.speed);
    SMO_Reset(&g_foc.ctrl.smo);
    FOC_Startup_Reset();
    FOC_ResetStallDetector();

    g_foc.status.run_counter = 0;
    g_foc.zc.debounce_cnt = 0;
    g_foc.zc.lockout_cnt = 0;
    g_foc.zc.grace_counter = 0;
    g_foc.status.state = FOC_STATE_IDLE;
}

NOINLINE_FUNC static void FOC_StateIdle(void) {
    FOC_EnableDrivers(0);

    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;

    g_foc.data.Ialpha_flt = 0.0f;
    g_foc.data.Ibeta_flt = 0.0f;
}

NOINLINE_FUNC static void FOC_StateStop(void) {
    FOC_EnableDrivers(0);
    if (g_foc.cfg.fault_oc_threshold > 0.001f) {
        FOC_HW_EnableAWD();
    }
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
    if (g_foc.status.state != FOC_STATE_FAULT) {
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

NOINLINE_FUNC static void FOC_StateFault(void) {
    FOC_EnableDrivers(0);
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
}

/*===========================================================================*/
/* Main ISR Entry Point                                                      */
/*===========================================================================*/

CCMRAM_FUNC void FOC_HighFrequencyTask(uint16_t adc1_data, uint16_t adc2_data) {
    if (!foc_initialized) return;
    if (g_foc.status.state != FOC_STATE_RUN) {
        g_foc.ctrl.smo.enable_harmonic_comp = 0;
    }

    /* Demux ADC data based on which phase was configured last ISR */
    uint16_t adc_ia = 0, adc_ib = 0, adc_ic = 0;

    switch (s_skip_phase) {
        case 0:
            adc_ib = adc2_data; /* ADC2 measured Phase B */
            adc_ic = adc1_data; /* ADC1 measured Phase C (PB1) */
            g_foc.data.Ib = ((float)adc_ib - (float)g_foc.cfg.cal_offset_b) * g_foc.data.i_scale;
            g_foc.data.Ic =
                ((float)adc_ic - (float)g_foc.cfg.cal_offset_c_pb1) * g_foc.data.i_scale;
            g_foc.data.Ia = -(g_foc.data.Ib + g_foc.data.Ic);
            break;
        case 1:
            adc_ia = adc1_data; /* ADC1 measured Phase A */
            adc_ic = adc2_data; /* ADC2 measured Phase C (VOPAMP3) */
            g_foc.data.Ia = ((float)adc_ia - (float)g_foc.cfg.cal_offset_a) * g_foc.data.i_scale;
            g_foc.data.Ic =
                ((float)adc_ic - (float)g_foc.cfg.cal_offset_c_op3) * g_foc.data.i_scale;
            g_foc.data.Ib = -(g_foc.data.Ia + g_foc.data.Ic);
            break;
        default:
            adc_ia = adc1_data; /* ADC1 measured Phase A */
            adc_ib = adc2_data; /* ADC2 measured Phase B */
            g_foc.data.Ia = ((float)adc_ia - (float)g_foc.cfg.cal_offset_a) * g_foc.data.i_scale;
            g_foc.data.Ib = ((float)adc_ib - (float)g_foc.cfg.cal_offset_b) * g_foc.data.i_scale;
            g_foc.data.Ic = -(g_foc.data.Ia + g_foc.data.Ib);
            break;
    }

    g_foc.status.run_counter++;

    if (g_foc.cfg.direction < 0.0f) {
        float t = g_foc.data.Ib;
        g_foc.data.Ib = g_foc.data.Ic;
        g_foc.data.Ic = t;
    }

    clarke_transform(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Ic, &g_foc.data.Ialpha,
                     &g_foc.data.Ibeta);

    stf_filter_step(g_foc.data.Ialpha, g_foc.data.Ibeta, &g_foc.data.Ialpha_flt,
                    &g_foc.data.Ibeta_flt, g_foc.wc_current_stf, g_foc.data.omega_elec, g_foc.dt);

    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_FAULT) {
        if (g_foc.status.state == FOC_STATE_IDLE) {
            FOC_StateIdle();
        } else {
            FOC_StateFault();
        }

        Comm_StreamPush();
        FOC_TriggerRegularADC();
        return;
    }
    if (g_foc.status.state == FOC_STATE_DETECT || g_foc.status.state == FOC_STATE_FLYING_START ||
        g_foc.status.state == FOC_STATE_COAST_FLUX_ID) {
        float va = foc_adc_to_vphase(adc_regular_buffer[0], g_foc.cfg.cal_vphase_a);
        float vc = foc_adc_to_vphase(adc_regular_buffer[1], g_foc.cfg.cal_vphase_c);
        float vb = -(va + vc);

        g_foc.data.Vphase_a = va;
        g_foc.data.Vphase_b = vb;
        g_foc.data.Vphase_c = vc;
    }

    FOC_TriggerRegularADC();

    switch (g_foc.status.state) {
        case FOC_STATE_IDLE:
            FOC_StateIdle();
            break;
        case FOC_STATE_CALIBRATION:
            FOC_Calibration_Accumulate(adc1_data, adc2_data, s_cal_phase);
            FOC_StateCalibration();
            break;
        case FOC_STATE_DETECT:
            FOC_StateDetect();
            break;
        case FOC_STATE_FLYING_START:
            FOC_StateFlyingStart();
            break;
        case FOC_STATE_ALIGN:
            FOC_StateAlign();
            break;
        case FOC_STATE_STARTUP:
            FOC_StateStartup();
            break;
        case FOC_STATE_RUN:
#if DEBUG_RUN_TIMEOUT_MS > 0
            if (g_foc.status.run_counter >=
                (uint32_t)(DEBUG_RUN_TIMEOUT_MS * 0.001f * g_foc.cfg.pwm_frequency)) {
                g_foc.status.state = FOC_STATE_STOP;
                break;
            }
#endif
            FOC_StateRun();
            break;

        case FOC_STATE_ZERO_CROSS:
            FOC_StateZeroCross();
            break;

        case FOC_STATE_BEEP:
            FOC_StateBeep();
            if (g_foc.status.state != FOC_STATE_BEEP) {
                return;
            }
            break;

        case FOC_STATE_STOP:
            FOC_StateStop();
            return;

        case FOC_STATE_FAULT:
            FOC_StateFault();
            return;

        case FOC_STATE_SELF_COMMISSION:
            FOC_StateSelfCommission();
            break;

        case FOC_STATE_COAST_FLUX_ID:
            FOC_StateCoastFluxID();
            Comm_StreamPush();
            return;
    }

    /* Continuous streaming telemetry */
    Comm_StreamPush();

    float out_a = g_foc.data.duty_a;
    float out_b = g_foc.data.duty_b;
    float out_c = g_foc.data.duty_c;

    foc_apply_deadtime_compensation(&out_a, &out_b, &out_c);

    /*=======================================================================*/
    /* ADC Channel Switching for NEXT trigger (duty-based + hysteresis)      */
    /*=======================================================================*/
    uint8_t do_switch_sensing = 0;
    uint8_t next_sensing_phase = s_skip_phase;

    if (g_foc.status.state == FOC_STATE_RUN || g_foc.status.state == FOC_STATE_ZERO_CROSS) {
        /* Map duties to physical inverter bridges A, B, C (accounts for reverse direction) */
        float phys_duties[3];
        phys_duties[0] = out_a;
        if (g_foc.cfg.direction > 0.0f) {
            phys_duties[1] = out_b;
            phys_duties[2] = out_c;
        } else {
            phys_duties[1] = out_c;
            phys_duties[2] = out_b;
        }

        /* Find phase with true maximum physical duty */
        uint8_t max_phase = 0;
        float max_duty = phys_duties[0];
        if (phys_duties[1] > max_duty) {
            max_duty = phys_duties[1];
            max_phase = 1;
        }
        if (phys_duties[2] > max_duty) {
            max_duty = phys_duties[2];
            max_phase = 2;
        }

        if (max_phase != s_skip_phase && max_duty > phys_duties[s_skip_phase] + SKIP_HYSTERESIS) {
            s_skip_phase = max_phase;
            next_sensing_phase = s_skip_phase;
            do_switch_sensing = 1;
        }
    } else if (g_foc.status.state == FOC_STATE_SELF_COMMISSION) {
        if (s_skip_phase != 2) {
            s_skip_phase = 2;
            next_sensing_phase = 2;
            do_switch_sensing = 1;
        }
    } else if (g_foc.status.state == FOC_STATE_CALIBRATION) {
        s_cal_phase = (s_cal_phase == 2) ? 0 : (s_cal_phase + 1);
        next_sensing_phase = s_cal_phase;
        do_switch_sensing = 1;
    }

    if (do_switch_sensing) {
        FOC_HW_SwitchCurrentSensing(next_sensing_phase);
    }

    if (g_foc.cfg.direction > 0.0f) {
        FOC_HW_SetPWMDuty(out_a, out_b, out_c);
    } else {
        FOC_HW_SetPWMDuty(out_a, out_c, out_b);
    }

    /* 2-cycle pipeline voltage tracking for Peak update:
     * Shift applied voltage so SMO receives the exact voltage applied during [-1.0 dt, 0] */
    g_foc.data.Valpha_applied = s_valpha_active;
    g_foc.data.Vbeta_applied = s_vbeta_active;
    s_valpha_active = g_foc.data.Valpha;
    s_vbeta_active = g_foc.data.Vbeta;
}

/*===========================================================================*/
/* State Implementations                                                     */
/*===========================================================================*/

NOINLINE_FUNC static void FOC_ZeroCross_Init(float target_dir, float omega_cross) {
    g_foc.zc.bridge_dir = target_dir;
    g_foc.zc.bridge_omega = g_foc.data.omega_elec;
    g_foc.zc.bridge_theta = g_foc.data.theta_park;
    g_foc.zc.bridge_target_omega = target_dir * omega_cross;
    g_foc.zc.id_current = 0.0f; /* Start Id from 0 for soft magnetic well entry */
    g_foc.zc.iq_current = g_foc.data.Iq;

    float cross_ratio = clampf(g_foc.cfg.cross_current_ratio, 0.10f, 1.0f);
    float safe_max_curr = fminf(g_foc.cfg.motor_max_curr, BOARD_MAX_CURRENT);
    float i_cross = cross_ratio * safe_max_curr;
    float b0 = g_foc.cfg.ladrc_b0;
    if (b0 < 10.0f) {
        b0 = g_foc.ctrl.speed.b0;
    }
    if (b0 < 10.0f) {
        b0 = 1000.0f;
    }

    float calc_slew = 0.50f * b0 * (0.7071f * i_cross);
    g_foc.zc.bridge_slew_rate = clampf(calc_slew, 500.0f, 100000.0f);
}

CCMRAM_FUNC static void FOC_StateZeroCross(void) {
    g_foc.zc.bridge_omega += g_foc.zc.bridge_dir * g_foc.zc.bridge_slew_rate * g_foc.dt;
    g_foc.zc.bridge_theta = normalize_angle_norm(g_foc.zc.bridge_theta +
                                                 (g_foc.zc.bridge_omega * (1.0f / PI)) * g_foc.dt);

    g_foc.data.theta_park = g_foc.zc.bridge_theta;
    float theta_advance = (g_foc.zc.bridge_omega * (1.0f / PI)) * (1.5f * g_foc.dt);
    g_foc.data.theta_elec = normalize_angle_norm(g_foc.zc.bridge_theta + theta_advance);
    g_foc.data.omega_elec = g_foc.zc.bridge_omega;

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_park, &cos_th, &sin_th);
    park_transform(g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    float cross_ratio = clampf(g_foc.cfg.cross_current_ratio, 0.10f, 1.0f);
    float safe_max_curr = fminf(g_foc.cfg.motor_max_curr, BOARD_MAX_CURRENT);
    float i_cross = cross_ratio * safe_max_curr;

    float target_id = 0.7071f * i_cross;
    float target_iq = g_foc.zc.bridge_dir * 0.7071f * i_cross;

    /* Soft magnetic well ramp (1.5ms rise time: no abrupt step) */
    float id_ramp_step = (target_id / 0.0015f) * g_foc.dt;
    if (g_foc.zc.id_current < target_id) {
        g_foc.zc.id_current += id_ramp_step;
        if (g_foc.zc.id_current > target_id) g_foc.zc.id_current = target_id;
    }
    g_foc.cmd.Id_ref = g_foc.zc.id_current;

    /* Smooth ramp for Iq */
    float iq_ramp_step = (i_cross / 0.0020f) * g_foc.dt;
    float iq_err = target_iq - g_foc.zc.iq_current;
    g_foc.zc.iq_current += clampf(iq_err, -iq_ramp_step, iq_ramp_step);
    g_foc.cmd.Iq_ref = g_foc.zc.iq_current;

    float l_ratio = g_foc.ctrl.smo.l_ratio;
    g_foc.ctrl.id.Kp = g_foc.cfg.kp_id * l_ratio;
    g_foc.ctrl.iq.Kp = g_foc.cfg.kp_iq * l_ratio;

    float ff_gain = 0.9f;
    float omega_Ls = ff_gain * g_foc.data.omega_elec * (g_foc.cfg.motor_ls * l_ratio);
    float E_bemf = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_flux;
    float max_v = SQRT3_INV * 2.0f * (g_foc.max_duty - 0.5f) * g_foc.data.Vbus;

    /* Enforce strict d-axis voltage headroom limit to prevent q-axis starvation */
    g_foc.ctrl.id.out_max = max_v;
    g_foc.ctrl.id.out_min = -max_v;
    PI_SetIntLimits(&g_foc.ctrl.id, -max_v, max_v);

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    g_foc.data.Vd =
        saturatef(PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq, max_v);

    /* Guaranteed q-axis voltage headroom: vq_max >= 0.866 * max_v */
    float vq_max_sq = max_v * max_v - g_foc.data.Vd * g_foc.data.Vd;
    float vq_max = (vq_max_sq > 0.0f) ? sqrtf(vq_max_sq) : 0.0f;

    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;
    float ff_q = omega_Ls * g_foc.data.Id + E_bemf;
    g_foc.ctrl.iq.out_max = vq_max - ff_q;
    g_foc.ctrl.iq.out_min = -vq_max - ff_q;
    PI_SetIntLimits(&g_foc.ctrl.iq, -vq_max - ff_q, vq_max - ff_q);
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + ff_q;

    svpwm_calculate(g_foc.data.theta_elec);

    uint8_t has_crossed =
        (g_foc.zc.bridge_dir < 0.0f && g_foc.zc.bridge_omega <= g_foc.zc.bridge_target_omega) ||
        (g_foc.zc.bridge_dir > 0.0f && g_foc.zc.bridge_omega >= g_foc.zc.bridge_target_omega);

    if (has_crossed) {
        SMO_ResetStatesWithAngle(&g_foc.ctrl.smo, g_foc.zc.bridge_omega, g_foc.zc.bridge_theta);
        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            g_foc.cmd.speed_ref = g_foc.zc.bridge_omega;
            LADRC_SeedState(&g_foc.ctrl.speed, g_foc.zc.bridge_omega, 0.0f);
        } else if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            float Vq_norm = (max_v > 1.0f) ? (g_foc.data.Vq / max_v) : 0.0f;
            g_foc.cmd.Vq_ref = clampf(Vq_norm, -1.0f, 1.0f);
            g_foc.ctrl.iq.integral = g_foc.data.Vq;
        }

        g_foc.startup.theta = g_foc.zc.bridge_theta;
        g_foc.startup.omega = g_foc.zc.bridge_omega;
        float slew_k = g_foc.zc.bridge_slew_rate * 0.001f;
        float trans_ms = (slew_k > 10.0f) ? clampf(120.0f / slew_k, 2.5f, 6.0f) : 6.0f;
        FOC_Startup_StartTransition(trans_ms, g_foc.zc.id_current);

        /* Restore d-axis limits for normal run */
        g_foc.ctrl.id.out_max = max_v;
        g_foc.ctrl.id.out_min = -max_v;
        PI_SetIntLimits(&g_foc.ctrl.id, -max_v, max_v);

        g_foc.status.state = FOC_STATE_RUN;
        g_foc.zc.grace_counter = 120;
        g_foc.zc.lockout_cnt = (uint32_t)(g_foc.cfg.pwm_frequency / 3.0f);
        g_foc.zc.debounce_cnt = 0;
        g_foc.zc.was_reversing = 0;
    }
}

uint8_t FOC_IsReversalGraceActive(void) {
    if (g_foc.status.state == FOC_STATE_ZERO_CROSS || FOC_IsInTransition()) {
        return 1;
    }
    if (g_foc.zc.grace_counter > 0) {
        g_foc.zc.grace_counter--;
        return 1;
    }
    return 0;
}

CCMRAM_FUNC static void FOC_StateRun(void) {
    SMO_Update(&g_foc.ctrl.smo, g_foc.data.Valpha_applied, g_foc.data.Vbeta_applied,
               g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt);

    float smo_theta_park = SMO_GetParkAngle(&g_foc.ctrl.smo);
    float smo_theta_pwm = SMO_GetPWMAngle(&g_foc.ctrl.smo);
    float smo_omega = SMO_GetSpeed(&g_foc.ctrl.smo);

    g_foc.ctrl.smo.enable_harmonic_comp = 0;
    uint8_t enter_crossing = 0;
    float cross_speed = g_foc.cfg.motor_min_spd;

    float target_dir = g_foc.status.reverse;
    if (g_foc.status.control_mode == FOC_MODE_SPEED) {
        if (fabsf(g_foc.cmd.speed_ref_target) > 1e-5f) {
            target_dir = (g_foc.cmd.speed_ref_target > 0.0f) ? 1.0f : -1.0f;
        }
    } else if (g_foc.status.control_mode == FOC_MODE_TORQUE) {
        if (fabsf(g_foc.cmd.Iq_ref_target) > 1e-5f) {
            target_dir = (g_foc.cmd.Iq_ref_target > 0.0f) ? 1.0f : -1.0f;
        }
    } else if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
        if (fabsf(g_foc.cmd.Vq_ref_target) > 1e-5f) {
            target_dir = (g_foc.cmd.Vq_ref_target > 0.0f) ? 1.0f : -1.0f;
        }
    }

    /* Only track steady-state running direction when NOT actively requesting reversal.
     * During reversal, g_foc.status.reverse  must remain locked to the prior direction until
     * zero-crossing is explicitly completed! */
    float actual_dir = (g_foc.data.omega_elec >= 0.0f) ? 1.0f : -1.0f;
    if (actual_dir == target_dir && fabsf(g_foc.data.omega_elec) > cross_speed) {
        g_foc.status.reverse = actual_dir;
    }

    if (FOC_IsInTransition()) {
        FOC_Transition_Update(smo_theta_park, smo_theta_pwm, smo_omega);
        g_foc.zc.lockout_cnt = (uint32_t)(g_foc.cfg.pwm_frequency * 0.1f);
        g_foc.zc.debounce_cnt = 0;
        g_foc.status.reverse = (g_foc.data.omega_elec >= 0.0f) ? 1.0f : -1.0f;
    } else {
        g_foc.data.theta_park = smo_theta_park;
        g_foc.data.theta_elec = smo_theta_pwm;
        g_foc.data.omega_elec = smo_omega;
        if (g_foc.zc.lockout_cnt > 0) {
            g_foc.zc.lockout_cnt--;
            g_foc.zc.debounce_cnt = 0;
        } else {
            uint8_t rev_requested = (target_dir != g_foc.status.reverse);
            float zc_threshold = cross_speed;
            uint8_t spd_reached = (fabsf(g_foc.data.omega_elec) <= zc_threshold);
            if (rev_requested && spd_reached) {
                if (++g_foc.zc.debounce_cnt >= 16) {
                    enter_crossing = 1;
                    g_foc.zc.debounce_cnt = 0;
                }
            } else {
                g_foc.zc.debounce_cnt = 0;
            }

            if (enter_crossing) {
                FOC_ZeroCross_Init(target_dir, cross_speed);
                g_foc.status.reverse = target_dir;
                g_foc.status.state = FOC_STATE_ZERO_CROSS;
                FOC_StateZeroCross();
                return;
            }
        }
    }

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_park, &cos_th, &sin_th);

    park_transform(g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    if (g_foc.ctrl.profiler.mode != PROFILER_MODE_OFF) {
        Profiler_Update(&g_foc.ctrl.profiler, (float*)&g_foc.cmd.Iq_ref);
    }

    if (!g_foc.status.in_transition) {
        g_foc.cmd.Id_ref = 0.0f;
    }

    float l_ratio = g_foc.ctrl.smo.l_ratio;
    g_foc.ctrl.id.Kp = g_foc.cfg.kp_id * l_ratio;
    g_foc.ctrl.iq.Kp = g_foc.cfg.kp_iq * l_ratio;

    float ff_gain = 0.9f;
    float omega_Ls = ff_gain * g_foc.data.omega_elec * (g_foc.cfg.motor_ls * l_ratio);
    float E_bemf = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_flux;
    float ff_q = omega_Ls * g_foc.data.Id + E_bemf;
    float max_v = SQRT3_INV * 2.0f * (g_foc.max_duty - 0.5f) * g_foc.data.Vbus;
    g_foc.ctrl.id.out_max = max_v;
    g_foc.ctrl.id.out_min = -max_v;
    PI_SetIntLimits(&g_foc.ctrl.id, -max_v, max_v);

    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    g_foc.data.Vd =
        saturatef(PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq, max_v);

    float vq_max_sq = max_v * max_v - g_foc.data.Vd * g_foc.data.Vd;
    float vq_max = (vq_max_sq > 0.0f) ? sqrtf(vq_max_sq) : 0.0f;

    uint8_t is_reversing = (target_dir != g_foc.status.reverse);
    uint8_t rev_started = (is_reversing && !g_foc.zc.was_reversing);
    uint8_t rev_ended = (!is_reversing && g_foc.zc.was_reversing);
    g_foc.zc.was_reversing = is_reversing;
    if (rev_started) {
        if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            g_foc.cmd.Iq_ref = g_foc.data.Iq;
            g_foc.ctrl.iq.integral = g_foc.data.Vq - ff_q;
        }
    } else if (rev_ended) {
        if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            g_foc.ctrl.iq.integral = g_foc.data.Vq;
            if (max_v > 1.0f) {
                g_foc.cmd.Vq_ref = clampf(g_foc.data.Vq / max_v, -1.0f, 1.0f);
            }
        }
    }

    uint8_t in_transition = FOC_IsInTransition();
    static uint8_t s_was_transition = 0;
    if (s_was_transition && !in_transition) {
        if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
            float vq_norm = (max_v > 1.0f) ? (g_foc.data.Vq / max_v) : 0.0f;
            g_foc.cmd.Vq_ref = clampf(vq_norm, -1.0f, 1.0f);
            g_foc.ctrl.iq.integral = g_foc.data.Vq;
        }
    }
    s_was_transition = in_transition;

    if (is_reversing) {
        float i_brake_target =
            target_dir * g_foc.cfg.cross_current_ratio * g_foc.cfg.motor_max_curr;
        float iq_ramp_step = g_foc.cfg.current_ramp_rate * g_foc.dt;
        float ramp_error = i_brake_target - g_foc.cmd.Iq_ref;
        g_foc.cmd.Iq_ref += clampf(ramp_error, -iq_ramp_step, iq_ramp_step);
        g_foc.cmd.Iq_ref = saturatef(g_foc.cmd.Iq_ref, g_foc.cfg.motor_max_curr);
    } else if (g_foc.status.control_mode == FOC_MODE_TORQUE ||
               g_foc.status.control_mode == FOC_MODE_SPEED || in_transition) {
        float iq_ramp_step = 1000.0f * g_foc.dt;
        if (g_foc.status.control_mode == FOC_MODE_TORQUE) {
            iq_ramp_step = g_foc.cfg.current_ramp_rate * g_foc.dt;
        }
        float ramp_error = g_foc.cmd.Iq_ref_target - g_foc.cmd.Iq_ref;
        g_foc.cmd.Iq_ref += clampf(ramp_error, -iq_ramp_step, iq_ramp_step);
        g_foc.cmd.Iq_ref = saturatef(g_foc.cmd.Iq_ref, g_foc.cfg.motor_max_curr);
    }

    float iq_out_min, iq_out_max, vq_ff;
    if (g_foc.status.control_mode == FOC_MODE_VOLTAGE && !is_reversing && !in_transition) {
        float ramp_time_s = g_foc.cfg.voltage_ramp_rate_ms * 0.001f;
        if (ramp_time_s < 0.005f) ramp_time_s = 0.005f;
        float volt_ramp_step = g_foc.dt / ramp_time_s;
        float ramp_v_err = g_foc.cmd.Vq_ref_target - g_foc.cmd.Vq_ref;
        g_foc.cmd.Vq_ref += clampf(ramp_v_err, -volt_ramp_step, volt_ramp_step);
        float vq_cmd = g_foc.cmd.Vq_ref * max_v;
        float vq_cmd_sat = clampf(vq_cmd, -vq_max, vq_max);

        if (g_foc.data.Iq >= 0.0f) {
            g_foc.cmd.Iq_ref = g_foc.cfg.motor_max_curr;
            iq_out_min = -vq_max;
            iq_out_max = vq_cmd_sat;
        } else {
            g_foc.cmd.Iq_ref = -g_foc.cfg.motor_max_curr;
            iq_out_min = vq_cmd_sat;
            iq_out_max = vq_max;
        }
        vq_ff = 0.0f;
    } else {
        iq_out_min = -vq_max - ff_q;
        iq_out_max = vq_max - ff_q;
        vq_ff = ff_q;
    }

    g_foc.ctrl.iq.out_min = iq_out_min;
    g_foc.ctrl.iq.out_max = iq_out_max;
    PI_SetIntLimits(&g_foc.ctrl.iq, iq_out_min, iq_out_max);

    float Iq_error = g_foc.cmd.Iq_ref - g_foc.data.Iq;
    g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + vq_ff;

    svpwm_calculate(g_foc.data.theta_elec);
}

NOINLINE_FUNC static void FOC_StateSelfCommission(void) {
    g_foc.data.theta_elec = 0.0f;
    float cos_th = 1.0f;
    float sin_th = 0.0f;

    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    /* Run the d-axis AC parameter identification step (Fast Task) */
    MotorID_FastTask(g_foc.data.Id, &g_foc.data.Vd, &g_foc.data.Vq);

    svpwm_calculate(0.0f);

    if (id_result.state == MOTOR_ID_STATE_COMPLETE || id_result.state == MOTOR_ID_STATE_ERROR) {
        FOC_BeepType_t tone =
            (id_result.state == MOTOR_ID_STATE_COMPLETE) ? FOC_BEEP_SUCCESS : FOC_BEEP_ERROR;
        FOC_StateStop();
        FOC_PlayBeep(tone);
    }
}

/*===========================================================================*/
/* API Functions                                                             */
/*===========================================================================*/
CCMRAM_FUNC void FOC_SetSpeedRef(float speed_rad) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    g_foc.cmd.speed_ref_target = speed_rad;
}

CCMRAM_FUNC void FOC_SetTorqueRef(float torque_percent) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    float pct = clampf(torque_percent, -1.0f, 1.0f);
    g_foc.cmd.Iq_ref_target = pct * g_foc.cfg.motor_max_curr;
}

CCMRAM_FUNC void FOC_SetTorqueCurrent(float iq_target_amp) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    g_foc.cmd.Iq_ref_target =
        clampf(iq_target_amp, -g_foc.cfg.motor_max_curr, g_foc.cfg.motor_max_curr);
}

CCMRAM_FUNC void FOC_SetVoltageRef(float voltage_percent) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    float pct = clampf(voltage_percent, -1.0f, 1.0f);
    g_foc.cmd.Vq_ref_target = pct;
}

void FOC_ClearFault(void) {
    if (g_foc.status.state == FOC_STATE_FAULT) {
        FOC_HW_ClearAWDFlags();

        g_foc.status.fault = FOC_FAULT_NONE;
        g_foc.status.state = FOC_STATE_IDLE;
        FOC_ResetStallDetector();

        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        LADRC_Reset(&g_foc.ctrl.speed);
        SMO_Reset(&g_foc.ctrl.smo);
        FOC_Startup_Reset();
    }
}

void FOC_EnableDrivers(uint8_t enable) {
    if (enable) {
        FOC_HW_EnableDrivers();
    } else {
        FOC_HW_DisableDrivers();
    }
}

void FOC_EnableDriver(uint8_t phase, uint8_t enable) {
    FOC_HW_EnablePhase(phase, enable);
}

void FOC_StartSelfCommission(void) {
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_STOP) {
        MotorID_Start();
        FOC_Start();
        FOC_EnableDrivers(0);
    }
}

CCMRAM_FUNC uint8_t FOC_IsProfilerActive(void) {
    return (g_foc.ctrl.profiler.mode != PROFILER_MODE_OFF);
}

/*===========================================================================*/
/* ESC Beep Sequencer                                                        */
/*===========================================================================*/
static void FOC_Beep_SetIdleOutputs(void) {
    g_foc.data.Vd = 0.0f;
    g_foc.data.Vq = 0.0f;
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
}

void FOC_StateBeep(void) {
    float vd = 0.0f;
    float theta = 0.0f;
    FOC_BeepStepStatus_t step = FOC_Beep_Step(&vd, &theta);

    if (step == FOC_BEEP_STEP_TONE) {
        g_foc.data.Vd = vd;
        g_foc.data.Vq = 0.0f;
        svpwm_calculate(theta);
        return;
    }

    FOC_Beep_SetIdleOutputs();
    if (step == FOC_BEEP_STEP_DONE) {
        FOC_EnableDrivers(0);
        FOC_Beep_Stop();
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

void FOC_PlayBeep(FOC_BeepType_t type) {
    if (g_foc.cfg.beep_volume == 0U) {
        return;
    }
    if (g_foc.status.state != FOC_STATE_IDLE && g_foc.status.state != FOC_STATE_STOP) {
        return;
    }
    if (g_foc.status.fault != FOC_FAULT_NONE) {
        return;
    }
    if (!FOC_Beep_Start(type, g_foc.dt, g_foc.cfg.motor_rs, g_foc.cfg.motor_ls, g_foc.data.Vbus,
                        g_foc.deadtime_duty, g_foc.cfg.beep_volume)) {
        return;
    }

    FOC_Beep_SetIdleOutputs();
    FOC_HW_SetPWMDuty(0.5f, 0.5f, 0.5f);
    FOC_EnableDrivers(1);
    g_foc.status.state = FOC_STATE_BEEP;
}