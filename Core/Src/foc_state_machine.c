/**
 * @file foc_state_machine.c
 * @brief FOC Core State Machine, High Frequency Task (48kHz ISR) and APIs
 */

#include "foc_state_machine.h"

#include "comm_protocol.h"
#include "cordic_math.h"
#include "foc.h"
#include "foc_calibration.h"
#include "foc_config.h"
#include "foc_flying_start.h"
#include "foc_input.h"
#include "foc_startup.h"
#include "main.h"
#include "math.h"
#include "motor_id.h"
#include "peripheral_init.h"

/*===========================================================================*/
/* Global Instance                                                           */
/*===========================================================================*/

FOC_Control_t g_foc __attribute__((aligned(4))); /* aligned for efficient FPU access */
static volatile uint8_t foc_initialized = 0;

/* Stop ramp-down state */
static uint8_t s_stopping = 0;      /* 1 = ramp-down in progress */
static uint32_t s_stop_counter = 0; /* Timeout counter */
static float s_saved_speed_target = 0.0f;
static float s_saved_iq_target = 0.0f;

/**
 * @brief Tracks which phase each ADC was configured to sample.
 *        Set at END of each ISR for the NEXT trigger.
 *        0 = Phase A, 1 = Phase B, 2 = Phase C
 */
static uint8_t s_adc1_phase = 0; /* Default: ADC1 → Phase A (VOPAMP1) */
static uint8_t s_adc2_phase = 1; /* Default: ADC2 → Phase B (VOPAMP2) */
static uint8_t s_skip_phase = 2; /* Which phase is currently skipped (0=A, 1=B, 2=C) */

extern volatile uint32_t adc_isr_us;
extern volatile uint16_t adc_regular_buffer[2];
extern volatile float ADC_Vref;

/*===========================================================================*/
/* Private Functions Prototypes                                              */
/*===========================================================================*/

__attribute__((noinline)) static void FOC_StateIdle(void);
static void FOC_StateRun(void);
__attribute__((noinline)) static void FOC_StateStop(void);
__attribute__((noinline)) static void FOC_StateFault(void);
__attribute__((noinline)) static void FOC_StateSelfCommission(void);

/*===========================================================================*/
/* Public Functions                                                          */
/*===========================================================================*/

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

    g_foc.cmd.Id_ref = 0.0f;
    g_foc.cmd.Iq_ref = 0.0f;
    g_foc.cmd.Vq_ref = 0.0f;
    g_foc.cmd.Vq_ref_target = 0.0f;
    g_foc.cmd.speed_ref = 0.0f;
    g_foc.data.theta_elec = 0.0f;
    g_foc.data.omega_elec = 0.0f;
    g_foc.data.speed_rpm = 0.0f;
    g_foc.data.e_real_flt = 0.0f;
    g_foc.data.e_expect_flt = 0.0f;
    g_foc.status.in_transition = 0;

    g_foc.startup.theta = 0.0f;
    g_foc.startup.omega = 0.0f;
    g_foc.startup.counter = 0;

    g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;

    PI_Init(&g_foc.ctrl.id, PI_ID_KP, PI_ID_KI, -12.0f, 12.0f, CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.id, -12.0f, 12.0f);

    PI_Init(&g_foc.ctrl.iq, PI_IQ_KP, PI_IQ_KI, -12.0f, 12.0f, CONTROL_PERIOD);
    PI_SetIntLimits(&g_foc.ctrl.iq, -12.0f, 12.0f);

    LADRC_Init(&g_foc.ctrl.speed, LADRC_OMEGA_C_DEFAULT, LADRC_OMEGA_O_DEFAULT, LADRC_B0_DEFAULT,
               PI_SPEED_OUT_MIN, PI_SPEED_OUT_MAX, 0.001f);

    SMO_Init(&g_foc.ctrl.smo);

    BIST_Init(&g_foc.ctrl.bist);

    g_foc.adc_cal.offset_a = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_b = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_c = ADC_CURRENT_OFFSET;
    g_foc.adc_cal.offset_vphase_a = 234;
    g_foc.adc_cal.offset_vphase_b = 234;
    g_foc.adc_cal.offset_vphase_c = 234;
    g_foc.adc_cal.cal_samples = 0;

    g_foc.status.run_counter = 0;
    g_foc.isr_time_cycles = 0;

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

        s_stopping = 0;
        s_stop_counter = 0;

        FOC_FlyingStart_Init();
        FOC_Calibration_Reset();

        /* Reset ADC channel switching to default (skip C) */
        s_skip_phase = 2;
        s_adc1_phase = 0;
        s_adc2_phase = 1;
        LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
        LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
        LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);

        FOC_SetPhaseVoltageDMA(1);
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
    if (g_foc.status.state == FOC_STATE_STARTUP || 1) {  // STOP
        s_stopping = 0;
    } else if (g_foc.status.state == FOC_STATE_RUN) {
        s_saved_speed_target = g_foc.cmd.speed_ref_target;
        s_saved_iq_target = g_foc.cmd.Iq_ref_target;
        g_foc.cmd.speed_ref_target = 0.0f;
        g_foc.cmd.Iq_ref_target = 0.0f;
        g_foc.cmd.Vq_ref_target = 0.0f;
        s_stopping = 1;
        s_stop_counter = 0;
    }
    g_foc.status.state = FOC_STATE_STOP;
}

/*===========================================================================*/
/* Main ISR Entry Point                                                      */
/*===========================================================================*/
CCMRAM_FUNC void FOC_HighFrequencyTask(uint16_t adc1_data, uint16_t adc2_data) {
    if (!foc_initialized) return;
    if (g_foc.status.state != FOC_STATE_RUN) {
        g_foc.ctrl.smo.enable_harmonic_comp = 0;
    }

    float i_scale = g_foc.data.i_scale;
    if (i_scale == 0.0f) {
        float vref = ADC_Vref;
        if (vref < 1.0f) vref = 3.3f;
        i_scale = vref * (ADC_TO_CURRENT);
    }

    /* Demux ADC data based on which phase was configured last ISR */
    uint16_t adc_ia = 0, adc_ib = 0, adc_ic = 0;

    switch (s_adc1_phase) {
        case 0:
            adc_ia = adc1_data;
            break; /* ADC1 measured Phase A */
        case 2:
            adc_ic = adc1_data;
            break; /* ADC1 measured Phase C (PB1) */
        default:
            break;
    }
    switch (s_adc2_phase) {
        case 1:
            adc_ib = adc2_data;
            break; /* ADC2 measured Phase B */
        case 2:
            adc_ic = adc2_data;
            break; /* ADC2 measured Phase C (VOPAMP3) */
        default:
            break;
    }

    /* Convert measured phases to current, reconstruct missing via Kirchhoff */
    if (s_skip_phase != 0)
        g_foc.data.Ia = ((float)adc_ia - (float)g_foc.adc_cal.offset_a) * i_scale;
    if (s_skip_phase != 1)
        g_foc.data.Ib = ((float)adc_ib - (float)g_foc.adc_cal.offset_b) * i_scale;
    if (s_skip_phase != 2)
        g_foc.data.Ic = ((float)adc_ic - (float)g_foc.adc_cal.offset_c) * i_scale;

    switch (s_skip_phase) {
        case 0:
            g_foc.data.Ia = -(g_foc.data.Ib + g_foc.data.Ic);
            break;
        case 1:
            g_foc.data.Ib = -(g_foc.data.Ia + g_foc.data.Ic);
            break;
        default:
            g_foc.data.Ic = -(g_foc.data.Ia + g_foc.data.Ib);
            break;
    }

    // LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, adc1_data);
    g_foc.status.run_counter++;

    if (g_foc.status.reverse < 0.0f) {
        float t = g_foc.data.Ib;
        g_foc.data.Ib = g_foc.data.Ic;
        g_foc.data.Ic = t;
    }

    clarke_transform(g_foc.data.Ia, g_foc.data.Ib, g_foc.data.Ic, &g_foc.data.Ialpha,
                     &g_foc.data.Ibeta);

    float wc = TWO_PI * CURRENT_STF_FC;
    float a = wc * CONTROL_PERIOD;
    float b = g_foc.data.omega_elec * CONTROL_PERIOD;
    float D_inv = 1.0f / ((1.0f + a) * (1.0f + a) + b * b);
    float ri_alpha = g_foc.data.Ialpha_flt + a * g_foc.data.Ialpha;
    float ri_beta = g_foc.data.Ibeta_flt + a * g_foc.data.Ibeta;
    g_foc.data.Ialpha_flt = ((1.0f + a) * ri_alpha - b * ri_beta) * D_inv;
    g_foc.data.Ibeta_flt = (b * ri_alpha + (1.0f + a) * ri_beta) * D_inv;

    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_FAULT) {
        if (g_foc.status.state == FOC_STATE_IDLE) {
            FOC_StateIdle();
        } else {
            FOC_StateFault();
        }

        /* Telemetry snapshot sampling */
        Comm_ProcessSampling();

        foc_set_pwm_duty(0.5f, 0.5f, 0.5f);
        g_foc.status.run_counter++;
        return;
    }
    if (g_foc.status.state == FOC_STATE_DETECT || g_foc.status.state == FOC_STATE_FLYING_START ||
        g_foc.status.state == FOC_STATE_COAST_FLUX_ID) {
        g_foc.data.Vphase_a =
            foc_adc_to_vphase(adc_regular_buffer[0], g_foc.adc_cal.offset_vphase_a);
        g_foc.data.Vphase_c =
            foc_adc_to_vphase(adc_regular_buffer[1], g_foc.adc_cal.offset_vphase_c);
        g_foc.data.Vphase_b = -(g_foc.data.Vphase_a + g_foc.data.Vphase_c);
    }

    switch (g_foc.status.state) {
        case FOC_STATE_IDLE:
            FOC_StateIdle();
            break;
        case FOC_STATE_CALIBRATION:
            FOC_Calibration_Accumulate(adc1_data, adc2_data);
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
        case FOC_STATE_RUN:
#if DEBUG_RUN_TIMEOUT_MS > 0
            if (g_foc.status.run_counter >= (DEBUG_RUN_TIMEOUT_MS * CONTROL_FREQUENCY / 1000)) {
                g_foc.status.state = FOC_STATE_STOP;
                break;
            }
#endif
            if (g_foc.status.state == FOC_STATE_STARTUP) {
                FOC_StateStartup();
            } else {
                FOC_StateRun();
            }
            break;

        case FOC_STATE_STOP:
            if (s_stopping) {
                FOC_StateRun();
            } else {
                FOC_StateStop();
            }
            break;

        case FOC_STATE_FAULT:
            FOC_StateFault();
            break;

        case FOC_STATE_SELF_COMMISSION:
            FOC_StateSelfCommission();
            break;

        case FOC_STATE_COAST_FLUX_ID:
            FOC_StateCoastFluxID();
            break;
    }

    /* Telemetry snapshot sampling */
    Comm_ProcessSampling();

    float out_a = g_foc.data.duty_a;
    float out_b = g_foc.data.duty_b;
    float out_c = g_foc.data.duty_c;

    foc_apply_deadtime_compensation(&out_a, &out_b, &out_c);

    /*=======================================================================*/
    /* ADC Channel Switching for NEXT trigger (duty-based + hysteresis)      */
    /*=======================================================================*/
    if (g_foc.status.state != FOC_STATE_CALIBRATION) {
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

        /* Only switch skip phase if max_phase exceeds current skip phase by hysteresis */
        if (max_phase != s_skip_phase && max_duty > phys_duties[s_skip_phase] + SKIP_HYSTERESIS) {
            s_skip_phase = max_phase;
        }

        /* Apply channel configuration based on skip decision */
        switch (s_skip_phase) {
            case 0: /* Skip Physical Phase A (OPAMP1) → measure C(ADC1 via PB1) + B(ADC2 via
                       VOPAMP2) */
                LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_12);
                LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
                LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
                s_adc1_phase = 2;
                s_adc2_phase = 1;
                break;
            case 1: /* Skip Physical Phase B (OPAMP2) → measure A(ADC1 via VOPAMP1) + C(ADC2 via
                       VOPAMP3) */
                LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
                LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP3_ADC2);
                LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
                s_adc1_phase = 0;
                s_adc2_phase = 2;
                break;
            default: /* Skip Physical Phase C (OPAMP3) → measure A(ADC1 via VOPAMP1) + B(ADC2 via
                        VOPAMP2) */
                LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
                LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
                LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
                s_adc1_phase = 0;
                s_adc2_phase = 1;
                break;
        }
    } else {
        /* During calibration: configure ADC2 for the NEXT sample index (cal_samples) */
        if ((g_foc.adc_cal.cal_samples & 1) != 0) {
            /* Next ISR sample (odd) will measure Phase C */
            LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP3_ADC2);
        } else {
            /* Next ISR sample (even) will measure Phase B */
            LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
        }
    }

    if (g_foc.status.reverse > 0) {
        foc_set_pwm_duty(out_a, out_b, out_c);
    } else {
        foc_set_pwm_duty(out_a, out_c, out_b);
    }
}

/*===========================================================================*/
/* State Implementations                                                     */
/*===========================================================================*/

static void FOC_StateIdle(void) {
    // playTune();
    FOC_EnableDriver(1, 0);
    FOC_EnableDriver(2, 0);
    FOC_EnableDriver(3, 0);

    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;

    g_foc.data.Ialpha_flt = 0.0f;
    g_foc.data.Ibeta_flt = 0.0f;
}

CCMRAM_FUNC static void FOC_StateRun(void) {
    float speed_rpm = fabsf(SMO_GetSpeedRPM(&g_foc.ctrl.smo));
    if (speed_rpm >= g_foc.cfg.motor_min_spd) {
        g_foc.ctrl.smo.enable_harmonic_comp = 1;
    } else if (speed_rpm < g_foc.cfg.motor_min_spd * 0.8f) {
        g_foc.ctrl.smo.enable_harmonic_comp = 0;
    }

    SMO_Update(&g_foc.ctrl.smo, g_foc.data.Valpha, g_foc.data.Vbeta, g_foc.data.Ialpha_flt,
               g_foc.data.Ibeta_flt);

    float smo_theta_park = SMO_GetParkAngle(&g_foc.ctrl.smo);
    float smo_theta_pwm = SMO_GetPWMAngle(&g_foc.ctrl.smo);
    float smo_omega = SMO_GetSpeed(&g_foc.ctrl.smo);
    float smo_speed_rpm = SMO_GetSpeedRPM(&g_foc.ctrl.smo);

    float ed = g_foc.data.Vd - g_foc.ctrl.smo.Rs * g_foc.data.Id +
               smo_omega * g_foc.ctrl.smo.Ls * g_foc.data.Iq;
    float eq = g_foc.data.Vq - g_foc.ctrl.smo.Rs * g_foc.data.Iq -
               smo_omega * g_foc.ctrl.smo.Ls * g_foc.data.Id;
    float e_real = sqrtf(ed * ed + eq * eq);
    float e_expect = g_foc.cfg.motor_flux * fabsf(smo_omega);
    const float e_lpf_alpha = 100.0f * CONTROL_PERIOD;
    g_foc.data.e_real_flt += e_lpf_alpha * (e_real - g_foc.data.e_real_flt);
    g_foc.data.e_expect_flt += e_lpf_alpha * (e_expect - g_foc.data.e_expect_flt);

    if (FOC_IsInTransition()) {
        FOC_Transition_Update(smo_theta_park, smo_theta_pwm, smo_omega, smo_speed_rpm);
    } else {
        g_foc.data.theta_park = smo_theta_park;
        g_foc.data.theta_elec = smo_theta_pwm;
        g_foc.data.omega_elec = smo_omega;
        g_foc.data.speed_rpm = smo_speed_rpm;
    }

    float sin_th, cos_th;
    cordic_sincos(g_foc.data.theta_park, &cos_th, &sin_th);

    park_transform(g_foc.data.Ialpha_flt, g_foc.data.Ibeta_flt, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    if (g_foc.ctrl.bist.mode != BIST_MODE_OFF) {
        BIST_Update(&g_foc.ctrl.bist, (float*)&g_foc.cmd.Iq_ref);
    }

    if (!g_foc.status.in_transition) {
        g_foc.cmd.Id_ref = 0.0f;
    }
    float target_iq = saturatef(g_foc.cmd.Iq_ref, g_foc.cfg.motor_max_curr);
    float max_diq = 1000.0f * CONTROL_PERIOD;
    g_foc.data.Iq_ref_cmd += clampf(target_iq - g_foc.data.Iq_ref_cmd, -max_diq, max_diq);

    /* Real-time Magnetic Saturation Gain Scheduling (reuse l_ratio from SMO_Update) */
    float l_ratio = g_foc.ctrl.smo.l_ratio;
    g_foc.ctrl.id.Kp = g_foc.cfg.kp_id * l_ratio;
    g_foc.ctrl.iq.Kp = g_foc.cfg.kp_iq * l_ratio;

    float ff_gain = 0.9f;
    float omega_Ls = ff_gain * g_foc.data.omega_elec * (g_foc.cfg.motor_ls * l_ratio);
    float E_bemf = ff_gain * g_foc.data.omega_elec * g_foc.cfg.motor_flux;
    float max_v = SQRT3_INV * 2.0f * (MAX_DUTY_HIGH - 0.5f) * g_foc.data.Vbus;

    if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
        float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
        g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq;
        float vd_abs = fabsf(g_foc.data.Vd);
        if (vd_abs >= max_v) {
            g_foc.data.Vd = (g_foc.data.Vd > 0.0f) ? max_v : -max_v;
            g_foc.data.Vq = 0.0f;
        } else {
            float vq_max = sqrtf(max_v * max_v - g_foc.data.Vd * g_foc.data.Vd);
            float vq_cmd = g_foc.cmd.Vq_ref * max_v;
            g_foc.data.Vq = clampf(vq_cmd, -vq_max, vq_max);
        }
    } else {
        float Id_error = g_foc.cmd.Id_ref - g_foc.data.Id;
        float Iq_error = g_foc.data.Iq_ref_cmd - g_foc.data.Iq;

        /* 1. Calculate unconstrained Vd and apply limits */
        g_foc.data.Vd = PI_Update(&g_foc.ctrl.id, Id_error) - omega_Ls * g_foc.data.Iq;

        /* 2. Calculate available Vq margin based on SVPWM circle and Vd */
        float vd_clamped = saturatef(g_foc.data.Vd, max_v);
        float vq_max_sq = max_v * max_v - vd_clamped * vd_clamped;
        float vq_max_avail = (vq_max_sq > 0.0f) ? sqrtf(vq_max_sq) : 0.0f;

        /* 3. Dynamic PI Limits for Iq (accounting for feedforward terms) */
        float ff_q = omega_Ls * g_foc.data.Id + E_bemf;
        g_foc.ctrl.iq.out_max = vq_max_avail - ff_q;
        g_foc.ctrl.iq.out_min = -vq_max_avail - ff_q;

        /* 4. Update Iq PI controller with dynamic limits */
        g_foc.data.Vq = PI_Update(&g_foc.ctrl.iq, Iq_error) + ff_q;
    }

    float theta_adv = g_foc.data.theta_elec;
    if (g_foc.cfg.comp_delay_samples > 0.001f) {
        theta_adv += g_foc.data.omega_elec * (g_foc.cfg.comp_delay_samples * CONTROL_PERIOD) / PI;
        theta_adv = normalize_angle_norm(theta_adv);
    }

    svpwm_calculate(theta_adv);

    /*=======================================================================*/
    /* Ramp-down stop check                                                  */
    /*=======================================================================*/
    if (s_stopping) {
        s_stop_counter++;
        float abs_speed = fabsf(g_foc.data.speed_rpm);
        float abs_iq = fabsf(g_foc.data.Iq);
        uint32_t timeout = (uint32_t)(STOP_TIMEOUT_MS * 0.001f * (float)CONTROL_FREQUENCY);
        float stop_current_thr = g_foc.cfg.motor_max_curr * (STOP_CURRENT_PERCENT / 100.0f);

        if ((abs_speed < g_foc.cfg.motor_min_spd && abs_iq < stop_current_thr &&
             g_foc.cmd.speed_ref == 0.0f) ||
            s_stop_counter >= timeout) {
            FOC_EnableDrivers(0);
            g_foc.data.duty_a = g_foc.data.duty_b = g_foc.data.duty_c = 0.5f;
            g_foc.cmd.Iq_ref = 0.0f;
            g_foc.data.Iq_ref_cmd = 0.0f;
            g_foc.cmd.Vq_ref = 0.0f;
            g_foc.cmd.Vq_ref_target = 0.0f;
            g_foc.cmd.speed_ref = 0.0f;
            PI_Reset(&g_foc.ctrl.id);
            PI_Reset(&g_foc.ctrl.iq);
            LADRC_Reset(&g_foc.ctrl.speed);
            s_stopping = 0;
            s_stop_counter = 0;
            g_foc.cmd.speed_ref_target = s_saved_speed_target;
            g_foc.cmd.Iq_ref_target = s_saved_iq_target;
            g_foc.status.state = FOC_STATE_IDLE;
        }
    }
}

static void FOC_StateStop(void) {
    FOC_SetPhaseVoltageDMA(0);
    FOC_EnableDrivers(0);
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
    g_foc.data.Iq_ref_cmd = 0.0f;
    if (g_foc.status.state != FOC_STATE_FAULT) {
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

static void FOC_StateFault(void) {
    FOC_SetPhaseVoltageDMA(0);
    FOC_EnableDrivers(0);
    g_foc.data.duty_a = 0.5f;
    g_foc.data.duty_b = 0.5f;
    g_foc.data.duty_c = 0.5f;
}

static void FOC_StateSelfCommission(void) {
    g_foc.data.theta_elec = 0.0f;
    float cos_th = 1.0f;
    float sin_th = 0.0f;

    /* Reconstruct currents and transform to d-q frame */
    park_transform(g_foc.data.Ialpha, g_foc.data.Ibeta, cos_th, sin_th, &g_foc.data.Id,
                   &g_foc.data.Iq);

    /* Run the d-axis AC parameter identification step */
    MotorID_RunStep(g_foc.data.Id, g_foc.data.Iq, g_foc.data.Vbus, &g_foc.data.Vd, &g_foc.data.Vq);

    /* Inverse Park and SVPWM calculation */
    svpwm_calculate(0.0f);

    if (id_result.state == MOTOR_ID_STATE_COMPLETE) {
        MotorID_Stop();
        FOC_StateStop();
    } else if (id_result.state == MOTOR_ID_STATE_ERROR) {
        FOC_StateStop();
    }
}

/*===========================================================================*/
/* API Functions                                                             */
/*===========================================================================*/

void FOC_SetSpeedRef(float speed_rpm) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    g_foc.cmd.speed_ref_target = speed_rpm * RPM_TO_RAD * g_foc.cfg.motor_poles;
}

void FOC_SetTorqueRef(float torque_percent) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    g_foc.cmd.Iq_ref_target = (torque_percent / 100.0f) * g_foc.cfg.motor_max_curr;
}

void FOC_SetVoltageRef(float voltage_percent) {
    if (g_foc.status.state == FOC_STATE_STOP) return;
    float pct = clampf(voltage_percent, 0.0f, 100.0f);
    g_foc.cmd.Vq_ref_target = pct / 100.0f;
}

void FOC_SetControlMode(FOC_ControlMode_t mode) {
    g_foc.status.control_mode = mode;
}

FOC_State_t FOC_GetState(void) {
    return g_foc.status.state;
}

FOC_Fault_t FOC_GetFault(void) {
    return g_foc.status.fault;
}

void FOC_ClearFault(void) {
    if (g_foc.status.state == FOC_STATE_FAULT) {
        // Clear all hardware AWD flags before transitioning out of FAULT state
        LL_ADC_ClearFlag_AWD1(ADC1);
        LL_ADC_ClearFlag_AWD2(ADC1);
        LL_ADC_ClearFlag_AWD1(ADC2);
        LL_ADC_ClearFlag_AWD2(ADC2);

        g_foc.status.fault = FOC_FAULT_NONE;
        g_foc.status.state = FOC_STATE_IDLE;
    }
}

void FOC_EnableDrivers(uint8_t enable) {
    if (enable) {
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                          LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                          LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
        LL_TIM_EnableAllOutputs(TIM1);
    } else {
        LL_TIM_DisableAllOutputs(TIM1);
        LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                           LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                           LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    }
}

void FOC_EnableDriver(uint8_t phase, uint8_t enable) {
    switch (phase) {
        case 1:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
            }
            break;
        case 2:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
            }
            break;
        case 3:
            if (enable) {
                LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
                LL_TIM_EnableAllOutputs(TIM1);
            } else {
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
            }
            break;
    }
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