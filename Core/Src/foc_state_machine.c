/**
 * @file foc_state_machine.c
 * @brief FOC Core State Machine, High Frequency Task and APIs
 */

#include "foc_state_machine.h"

#include <math.h>

#include "comm_protocol.h"
#include "cordic_math.h"
#include "foc.h"
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

static void FOC_StateRun(void);
static void FOC_StateSelfCommission(void);

uint8_t FOC_IsInitialized(void) {
    return foc_initialized;
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
    g_foc.data.Iq_ref_cmd = 0.0f;
    g_foc.data.Valpha = g_foc.data.Vbeta = 0.0f;
    g_foc.data.Vbus = 12.0f;
    g_foc.data.Ibus = 0.0f;
    g_foc.data.inv_i_th = 20.0f;

    g_foc.cmd.Id_ref = 0.0f;
    g_foc.cmd.Iq_ref = 0.0f;
    g_foc.cmd.Vq_ref = 0.0f;
    g_foc.cmd.Vq_ref_target = 0.0f;
    g_foc.cmd.speed_ref = 0.0f;
    g_foc.data.theta_elec = 0.0f;
    g_foc.data.omega_elec = 0.0f;
    g_foc.data.e_real_flt = 0.0f;
    g_foc.data.e_expect_flt = 0.0f;
    g_foc.status.in_transition = 0;

    g_foc.startup.theta = 0.0f;
    g_foc.startup.omega = 0.0f;
    g_foc.startup.counter = 0;

    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    g_foc.dt = (g_foc.cfg.pwm_frequency > 0.0f) ? (1.0f / g_foc.cfg.pwm_frequency) : CONTROL_PERIOD;

    PI_Init(&g_foc.ctrl.id, PI_ID_KP, PI_ID_KI, -12.0f, 12.0f, g_foc.dt);
    PI_SetIntLimits(&g_foc.ctrl.id, -12.0f, 12.0f);

    PI_Init(&g_foc.ctrl.iq, PI_IQ_KP, PI_IQ_KI, -12.0f, 12.0f, g_foc.dt);
    PI_SetIntLimits(&g_foc.ctrl.iq, -12.0f, 12.0f);

    LADRC_Init(&g_foc.ctrl.speed, LADRC_OMEGA_C_DEFAULT, LADRC_OMEGA_O_DEFAULT, LADRC_B0_DEFAULT,
               PI_SPEED_OUT_MIN, PI_SPEED_OUT_MAX, 0.001f);

    SMO_Init(&g_foc.ctrl.smo);

    Profiler_Init(&g_foc.ctrl.profiler);

    g_foc.adc_cal.offset_a = 2004;
    g_foc.adc_cal.offset_b = 2014;
    g_foc.adc_cal.offset_c_pb1 = 2007;
    g_foc.adc_cal.offset_c_opamp3 = 2010;
    g_foc.adc_cal.offset_vphase_a = 325;
    g_foc.adc_cal.offset_vphase_b = 234;
    g_foc.adc_cal.offset_vphase_c = 323;
    g_foc.adc_cal.cal_samples = 0;
    g_foc.noise_profile.noise_rms = 0.100f;
    g_foc.cfg.motor_min_spd = FOC_CalculateObserverMinSpeed();

    g_foc.status.run_counter = 0;
    g_foc.isr_time_cycles = 0;
    g_foc.data.i_scale = ADC_Vref * (ADC_TO_CURRENT);
    FOC_EnableDrivers(0);

    MotorID_Init();

    g_foc.status.reverse = 1.0f;

    g_foc.cfg.startup_current = STARTUP_CURRENT;
    g_foc.cfg.align_current = ALIGN_CURRENT;
    g_foc.cfg.startup_accel = STARTUP_ACCEL;
    g_foc.cfg.startup_handoff_speed = STARTUP_HANDOFF_SPEED;
    g_foc.cfg.speed_ramp_accel = SPEED_RAMP_ACCEL;
    g_foc.cfg.speed_ramp_decel = SPEED_RAMP_DECEL;
    g_foc.cfg.ladrc_omega_c = LADRC_OMEGA_C_DEFAULT;
    g_foc.cfg.ladrc_omega_o = LADRC_OMEGA_O_DEFAULT;
    g_foc.cfg.ladrc_b0 = LADRC_B0_DEFAULT;
    g_foc.cfg.motor_inertia = MOTOR_INERTIA;
    g_foc.cfg.fault_oc_threshold = FAULT_OVERCURRENT_THRESHOLD;
    g_foc.cfg.fault_oc_count = (uint8_t)FAULT_OVERCURRENT_COUNT;
    g_foc.cfg.fault_ov_threshold = FAULT_OVERVOLTAGE_THRESHOLD;
    g_foc.cfg.fault_uv_threshold = FAULT_UNDERVOLTAGE_THRESHOLD;
    g_foc.cfg.fault_stall_enable = (uint8_t)FAULT_STALL_ENABLE;
    g_foc.cfg.fault_stall_speed = FAULT_STALL_SPEED_RPM;
    g_foc.cfg.fault_stall_current = FAULT_STALL_CURRENT_A;
    g_foc.cfg.fault_stall_time_ms = FAULT_STALL_TIME_MS;

    foc_initialized = 1;
    FOC_ResetStallDetector();
    FOC_Input_Init();
}

void FOC_Start(void) {
    if (g_foc.status.state == FOC_STATE_IDLE) {
        PI_Reset(&g_foc.ctrl.id);
        PI_Reset(&g_foc.ctrl.iq);
        LADRC_Reset(&g_foc.ctrl.speed);
        SMO_Reset(&g_foc.ctrl.smo);

        /* Ensure PI controllers use original default gains from config */
        PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
        PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);

        FOC_Startup_Reset();
        g_foc.data.Iq_ref_cmd = 0.0f;

        FOC_FlyingStart_Init();
        FOC_Calibration_Reset();

        /* Reset ADC channel switching to default (skip C) */
        s_skip_phase = 2;
        s_cal_phase = 2;
        FOC_HW_SwitchCurrentSensing(s_skip_phase);

        g_foc.status.state = FOC_STATE_CALIBRATION;

        float v_limit = g_foc.data.Vbus * SQRT3_INV;
        PI_SetLimits(&g_foc.ctrl.id, -v_limit, v_limit);
        PI_SetIntLimits(&g_foc.ctrl.id, -v_limit, v_limit);
        PI_SetLimits(&g_foc.ctrl.iq, -v_limit, v_limit);
        PI_SetIntLimits(&g_foc.ctrl.iq, -v_limit, v_limit);
        g_foc.data.e_real_flt = 0.0f;
        g_foc.data.e_expect_flt = 0.0f;
    }
}

void FOC_Stop(void) {
    FOC_EnableDrivers(0);
    MotorID_Stop();
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
    g_foc.status.state = FOC_STATE_IDLE;
}

static inline void FOC_StateIdle(void) {
    FOC_EnableDrivers(0);

    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;

    g_foc.data.Ialpha_flt = 0.0f;
    g_foc.data.Ibeta_flt = 0.0f;
}

static inline void FOC_StateStop(void) {
    FOC_EnableDrivers(0);
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
    if (g_foc.status.state != FOC_STATE_FAULT) {
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

static inline void FOC_StateFault(void) {
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
            g_foc.data.Ib = ((float)adc_ib - (float)g_foc.adc_cal.offset_b) * g_foc.data.i_scale;
            g_foc.data.Ic =
                ((float)adc_ic - (float)g_foc.adc_cal.offset_c_pb1) * g_foc.data.i_scale;
            g_foc.data.Ia = -(g_foc.data.Ib + g_foc.data.Ic);
            break;
        case 1:
            adc_ia = adc1_data; /* ADC1 measured Phase A */
            adc_ic = adc2_data; /* ADC2 measured Phase C (VOPAMP3) */
            g_foc.data.Ia = ((float)adc_ia - (float)g_foc.adc_cal.offset_a) * g_foc.data.i_scale;
            g_foc.data.Ic =
                ((float)adc_ic - (float)g_foc.adc_cal.offset_c_opamp3) * g_foc.data.i_scale;
            g_foc.data.Ib = -(g_foc.data.Ia + g_foc.data.Ic);
            break;
        default:
            adc_ia = adc1_data; /* ADC1 measured Phase A */
            adc_ib = adc2_data; /* ADC2 measured Phase B */
            g_foc.data.Ia = ((float)adc_ia - (float)g_foc.adc_cal.offset_a) * g_foc.data.i_scale;
            g_foc.data.Ib = ((float)adc_ib - (float)g_foc.adc_cal.offset_b) * g_foc.data.i_scale;
            g_foc.data.Ic = -(g_foc.data.Ia + g_foc.data.Ib);
            break;
    }

    g_foc.status.run_counter++;

    if (g_foc.status.reverse < 0.0f) {
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
        float va = foc_adc_to_vphase(adc_regular_buffer[0], g_foc.adc_cal.offset_vphase_a);
        float vc = foc_adc_to_vphase(adc_regular_buffer[1], g_foc.adc_cal.offset_vphase_c);
        float vb = -(va + vc);

        g_foc.data.Vphase_a = va;
        g_foc.data.Vphase_b = vb;
        g_foc.data.Vphase_c = vc;
    }

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
        case FOC_STATE_BRAKE:
            FOC_StateBrake();
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

        case FOC_STATE_STOP:
            FOC_StateStop();
            FOC_TriggerRegularADC();
            return;

        case FOC_STATE_FAULT:
            FOC_StateFault();
            FOC_TriggerRegularADC();
            return;

        case FOC_STATE_SELF_COMMISSION:
            FOC_StateSelfCommission();
            break;

        case FOC_STATE_COAST_FLUX_ID:
            FOC_StateCoastFluxID();
            Comm_StreamPush();
            FOC_TriggerRegularADC();
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
    if (g_foc.status.state != FOC_STATE_CALIBRATION &&
        g_foc.status.state != FOC_STATE_SELF_COMMISSION) {
        /* Map duties to physical inverter bridges A, B, C (accounts for reverse direction) */
        float phys_duties[3];
        phys_duties[0] = out_a;
        if (g_foc.status.reverse > 0.0f) {
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
            FOC_HW_SwitchCurrentSensing(s_skip_phase);
        }
    } else if (g_foc.status.state == FOC_STATE_SELF_COMMISSION) {
        if (s_skip_phase != 2) {
            s_skip_phase = 2;
            FOC_HW_SwitchCurrentSensing(2);
        }
    } else {
        s_cal_phase = (s_cal_phase == 2) ? 0 : (s_cal_phase + 1);
        FOC_HW_SwitchCurrentSensing(s_cal_phase);
    }

    if (g_foc.status.reverse > 0) {
        FOC_HW_SetPWMDuty(out_a, out_b, out_c);
    } else {
        FOC_HW_SetPWMDuty(out_a, out_c, out_b);
    }
    FOC_TriggerRegularADC();
}

/*===========================================================================*/
/* State Implementations                                                     */
/*===========================================================================*/

CCMRAM_FUNC static void FOC_StateRun(void) {
    SMO_Update(&g_foc.ctrl.smo, g_foc.data.Valpha, g_foc.data.Vbeta, g_foc.data.Ialpha_flt,
               g_foc.data.Ibeta_flt);

    float smo_theta_park = SMO_GetParkAngle(&g_foc.ctrl.smo);
    float smo_theta_pwm = SMO_GetPWMAngle(&g_foc.ctrl.smo);
    float smo_omega = SMO_GetSpeed(&g_foc.ctrl.smo);

    g_foc.ctrl.smo.enable_harmonic_comp = (fabsf(smo_omega) >= g_foc.cfg.motor_min_spd) ? 1 : 0;

    if (FOC_IsInTransition()) {
        FOC_Transition_Update(smo_theta_park, smo_theta_pwm, smo_omega);
    } else {
        g_foc.data.theta_park = smo_theta_park;
        g_foc.data.theta_elec = smo_theta_pwm;
        g_foc.data.omega_elec = smo_omega;
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

    if (g_foc.status.control_mode == FOC_MODE_TORQUE) {
        float ramp_rate =
            (g_foc.cfg.current_ramp_rate > 0.0f) ? g_foc.cfg.current_ramp_rate : 1000.0f;
        float iq_ramp_step = ramp_rate * g_foc.dt;
        float ramp_error = g_foc.cmd.Iq_ref_target - g_foc.cmd.Iq_ref;
        g_foc.cmd.Iq_ref += clampf(ramp_error, -iq_ramp_step, iq_ramp_step);
    }

    float target_iq = saturatef(g_foc.cmd.Iq_ref, g_foc.cfg.motor_max_curr);
    float max_diq = 1000.0f * g_foc.dt;
    g_foc.data.Iq_ref_cmd += clampf(target_iq - g_foc.data.Iq_ref_cmd, -max_diq, max_diq);

    float l_ratio = g_foc.ctrl.smo.l_ratio;
    g_foc.ctrl.id.Kp = g_foc.cfg.kp_id * l_ratio;
    g_foc.ctrl.iq.Kp = g_foc.cfg.kp_iq * l_ratio;

    float ff_gain = 0.9f;
    float omega_Ls = ff_gain * g_foc.data.omega_elec * (g_foc.cfg.motor_ls * l_ratio);
    float E_bemf = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_flux;
    float max_v = SQRT3_INV * 2.0f * (g_foc.max_duty - 0.5f) * g_foc.data.Vbus;

    /* 1. Common d-axis voltage calculation and SVPWM circle limit */
    float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
    g_foc.data.Vd =
        saturatef(PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq, max_v);

    /* 2. Common remaining voltage margin for q-axis */
    float vq_max_sq = max_v * max_v - g_foc.data.Vd * g_foc.data.Vd;
    float vq_max = (vq_max_sq > 0.0f) ? sqrtf(vq_max_sq) : 0.0f;

    /* 3. Compute q-axis voltage according to control mode */
    if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
        float volt_ramp_step = (g_foc.cfg.voltage_ramp_rate / max_v) * g_foc.dt;
        float ramp_error = g_foc.cmd.Vq_ref_target - g_foc.cmd.Vq_ref;
        g_foc.cmd.Vq_ref += clampf(ramp_error, -volt_ramp_step, volt_ramp_step);

        float vq_cmd = g_foc.cmd.Vq_ref * max_v;

        /* Back-EMF Voltage Floor: clamp regenerative braking current to
         * VOLTAGE_MODE_REGEN_CURRENT_MAX */
        float e_bemf_full = g_foc.data.omega_elec * g_foc.cfg.motor_flux;
        float vq_floor = e_bemf_full - (g_foc.cfg.motor_rs * VOLTAGE_MODE_REGEN_CURRENT_MAX);
        if (vq_floor < 0.0f) {
            vq_floor = 0.0f;
        }
        if (vq_cmd < vq_floor) {
            vq_cmd = vq_floor;
        }

        g_foc.data.Vq = clampf(vq_cmd, 0.0f, vq_max);
    } else {
        float Iq_error = g_foc.data.Iq_ref_cmd - g_foc.data.Iq;

        /* Dynamic PI Limits for Iq (accounting for feedforward terms) */
        float ff_q = omega_Ls * g_foc.data.Id + E_bemf;
        g_foc.ctrl.iq.out_max = vq_max - ff_q;
        g_foc.ctrl.iq.out_min = -vq_max - ff_q;

        /* Update Iq PI controller with dynamic limits */
        g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + ff_q;
    }

    svpwm_calculate(g_foc.data.theta_elec);
}

static void FOC_StateSelfCommission(void) {
    g_foc.data.theta_elec = 0.0f;
    float cos_th = 1.0f;
    float sin_th = 0.0f;

    /* Reconstruct currents and transform to d-q frame */
    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    /* Run the d-axis AC parameter identification step (Fast Task) */
    MotorID_FastTask(g_foc.data.Id, &g_foc.data.Vd, &g_foc.data.Vq);

    /* Inverse Park and SVPWM calculation */
    svpwm_calculate(0.0f);

    if (id_result.state == MOTOR_ID_STATE_COMPLETE || id_result.state == MOTOR_ID_STATE_ERROR) {
        FOC_StateStop();
    }
}

/*===========================================================================*/
/* API Functions                                                             */
/*===========================================================================*/

void FOC_SetSpeedRef(float speed_rad) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    g_foc.cmd.speed_ref_target = speed_rad;
}

void FOC_SetTorqueRef(float torque_percent) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    float pct = clampf(torque_percent, 0.0f, 100.0f);
    g_foc.cmd.Iq_ref_target = (pct / 100.0f) * g_foc.cfg.motor_max_curr;
}

void FOC_SetVoltageRef(float voltage_percent) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    float pct = clampf(voltage_percent, 0.0f, 100.0f);
    g_foc.cmd.Vq_ref_target = pct / 100.0f;
}

void FOC_SetControlMode(FOC_ControlMode_t mode) {
    g_foc.status.control_mode = mode;
}

void FOC_ClearFault(void) {
    if (g_foc.status.state == FOC_STATE_FAULT) {
        // Clear all hardware AWD flags before transitioning out of FAULT state
        FOC_HW_ClearAWDFlags();

        g_foc.status.fault = FOC_FAULT_NONE;
        g_foc.status.state = FOC_STATE_IDLE;
        FOC_ResetStallDetector();
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
        FOC_Start();
        MotorID_Start();
        FOC_EnableDrivers(0);
    }
}

void FOC_SetDirection(int8_t dir) {
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_STOP ||
        g_foc.status.state == FOC_STATE_FAULT) {
        g_foc.status.reverse = (dir < 0) ? -1.0f : 1.0f;
    }
}

int8_t FOC_GetDirection(void) {
    return (g_foc.status.reverse < 0.0f) ? -1 : 1;
}

float FOC_GetDt(void) {
    return g_foc.dt;
}

void playTune(void) {
    uint32_t cycle_tick = g_foc.status.run_counter % BEEP_PERIOD_TICKS;
    uint32_t b1_end = BEEP_DURATION_TICKS;
    uint32_t b2_start = BEEP_DURATION_TICKS * 2;
    uint32_t b2_end = BEEP_DURATION_TICKS * 3;

    if ((cycle_tick < b1_end) || (cycle_tick >= b2_start && cycle_tick < b2_end)) {
        static float tune_phase = 0.0f;
        tune_phase += BEEP_STEP_FREQ;
        if (tune_phase >= 1.0f) tune_phase -= 2.0f;

        float cos_out, sin_out;
        cordic_sincos(tune_phase, &cos_out, &sin_out);

        g_foc.data.duty_a = 0.5f + sin_out * 0.05f;
        g_foc.data.duty_b = 0.5f - sin_out * 0.05f;
        g_foc.data.duty_c = 0.5f;
    } else {
        g_foc.data.duty_a = 0.5f;
        g_foc.data.duty_b = 0.5f;
        g_foc.data.duty_c = 0.5f;
    }
}