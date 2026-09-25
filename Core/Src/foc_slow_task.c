/**
 * @file foc_slow_task.c
 * @brief FOC 1kHz Slow Task, Speed Loop, Ramps & Safety Protections Implementation
 */

#include "foc_slow_task.h"

#include <math.h>

#include "comm_protocol.h"
#include "cordic_math.h"
#include "foc_config.h"
#include "foc_input.h"
#include "foc_state_machine.h"
#include "ladrc_controller.h"
#include "motor_id.h"
#include "peripheral_init.h"

static uint32_t stall_counter = 0;       /* Stall timer (ticks) */
static uint32_t ground_fault_count = 0;  /* Ground fault deglitch counter */
static uint32_t current_sat_counter = 0; /* Sustained current saturation overload timer */

/* Multi-layer stall detector filter states */
static float s_p_em_flt = 0.0f;         /* Filtered electromechanical power [W] */
static float s_s_app_flt = 0.0f;        /* Filtered apparent power [VA] */
static float s_stall_risk_accum = 0.0f; /* Leaky stall risk accumulator [0.0 to 1.0] */

extern volatile float ADC_Vref;

void FOC_ResetStallDetector(void) {
    stall_counter = 0;
    s_p_em_flt = 0.0f;
    s_s_app_flt = 0.0f;
    s_stall_risk_accum = 0.0f;
    g_foc.data.stall_risk = 0.0f;
    g_foc.data.eta_em = 1.0f;
    g_foc.data.d_desync = 0.0f;
    g_foc.data.r_bemf = 1.0f;
}

void FOC_Safety() {
    /* Software OV/UV protection (replaces hardware AWD2) */
    if (g_foc.status.state == FOC_STATE_RUN || g_foc.status.state == FOC_STATE_STARTUP) {
        if (g_foc.data.Vbus > g_foc.cfg.fault_ov_threshold) {
            FOC_EnableDrivers(0);
            g_foc.status.fault = FOC_FAULT_OVERVOLTAGE;
            g_foc.status.state = FOC_STATE_FAULT;
        } else if (g_foc.data.Vbus < g_foc.cfg.fault_uv_threshold) {
            FOC_EnableDrivers(0);
            g_foc.status.fault = FOC_FAULT_UNDERVOLTAGE;
            g_foc.status.state = FOC_STATE_FAULT;
        }
    }

    /* Motor Identification 1kHz Slow Task */
    if (g_foc.status.state == FOC_STATE_SELF_COMMISSION) {
        MotorID_SlowTask();
    }

    if (g_foc.status.state == FOC_STATE_RUN) {
        /* 1. Ground fault protection */
        if (g_foc.data.duty_a < 0.85f && g_foc.data.duty_b < 0.85f && g_foc.data.duty_c < 0.85f &&
            g_foc.data.duty_a > 0.15f && g_foc.data.duty_b > 0.15f && g_foc.data.duty_c > 0.15f) {
            float current_sum = g_foc.data.Ia + g_foc.data.Ib + g_foc.data.Ic;
            float gf_threshold = 0.40f * g_foc.cfg.motor_max_curr;
            if (gf_threshold < 1.0f) gf_threshold = 1.0f;

            if (fabsf(current_sum) > gf_threshold) {
                if (++ground_fault_count >= 2) {
                    FOC_EnableDrivers(0);
                    g_foc.status.fault = FOC_FAULT_GROUND;
                    g_foc.status.state = FOC_STATE_FAULT;
                    return;
                }
            } else {
                if (ground_fault_count > 0) ground_fault_count--;
            }
        } else {
            if (ground_fault_count > 0) ground_fault_count--;
        }

        /* 2. Observer Integrity & Overspeed Protection (Layer 2) */
        if (fabsf(g_foc.data.omega_elec) > g_foc.cfg.motor_max_spd * 1.25f) {
            FOC_EnableDrivers(0);
            g_foc.status.fault = FOC_FAULT_OBSERVER_FAIL;
            g_foc.status.state = FOC_STATE_FAULT;
            return;
        }

        /* 3. Advanced 4-Layer Stall & Desynchronization Protection */
        if (g_foc.cfg.fault_stall_enable) {
            float i_max = g_foc.cfg.motor_max_curr;
            float v_bus = g_foc.data.Vbus;
            float rs = g_foc.cfg.motor_rs;
            float flux = g_foc.cfg.motor_flux;

            /* Adaptive minimum sensing speed: 70% of handoff speed */
            float handoff_omega = g_foc.cfg.startup_handoff_speed;
            if (handoff_omega < 70.0f) handoff_omega = 500.0f;
            float omega_stall_min_elec = 0.70f * handoff_omega;

            float i_stall_thr = 0.20f * i_max;
            if (i_stall_thr < 0.80f) i_stall_thr = 0.80f;
            if (g_foc.cfg.fault_stall_current >= 0.5f &&
                g_foc.cfg.fault_stall_current < i_stall_thr) {
                i_stall_thr = g_foc.cfg.fault_stall_current;
            }

            float s_floor = 0.03f * v_bus * i_max;
            float e_floor = 0.35f * (omega_stall_min_elec * flux);

            /* --- Layer 1: Universal Electromechanical Power Conversion Ratio (eta_em) --- */
            float p_in = 1.5f * (g_foc.data.Vd * g_foc.data.Id + g_foc.data.Vq * g_foc.data.Iq);
            float p_cu =
                1.5f * rs * (g_foc.data.Id * g_foc.data.Id + g_foc.data.Iq * g_foc.data.Iq);
            float p_em = p_in - p_cu;
            float v_mag = sqrtf(g_foc.data.Vd * g_foc.data.Vd + g_foc.data.Vq * g_foc.data.Vq);
            float i_mag = sqrtf(g_foc.data.Id * g_foc.data.Id + g_foc.data.Iq * g_foc.data.Iq);
            float s_app = 1.5f * v_mag * i_mag;

            /* 5 ms Low-Pass Filter (alpha = dt / (dt + tau) = 0.001 / 0.006 = 0.1667f) */
            s_p_em_flt += 0.1667f * (p_em - s_p_em_flt);
            s_s_app_flt += 0.1667f * (s_app - s_s_app_flt);

            float eta_em = (s_s_app_flt > s_floor) ? (s_p_em_flt / s_s_app_flt) : 0.50f;
            if (eta_em > 1.0f) eta_em = 1.0f;
            if (eta_em < -1.0f) eta_em = -1.0f;

            /* --- Layer 2: Vector Orthogonality Check (D_desync) --- */
            /* Read synchronous BEMF magnitude and PLL angle tracking error directly from ISR */
            float e_obs_mag = g_foc.ctrl.smo.bemf_mag;
            float d_desync =
                (e_obs_mag > e_floor) ? fabsf(sinf(g_foc.ctrl.smo.theta_err * PI)) : 0.0f;

            /* --- Layer 3: Back-EMF Residual Ratio (R_bemf / Hallucination Buster) --- */
            float speed_elec_abs = fabsf(g_foc.data.omega_elec);
            float e_expect = flux * speed_elec_abs;
            float r_bemf = 1.0f;
            if (speed_elec_abs > omega_stall_min_elec) {
                float denom = (e_expect > e_floor) ? e_expect : e_floor;
                r_bemf = e_obs_mag / denom;
                if (r_bemf > 1.0f) r_bemf = 1.0f;
            }

            /* Record real-time diagnostic telemetry */
            g_foc.data.eta_em = eta_em;
            g_foc.data.d_desync = d_desync;
            g_foc.data.r_bemf = r_bemf;
            g_foc.data.stall_risk = s_stall_risk_accum;

            /* --- Layer 4: Multi-Condition Synthesis & Leaky Risk Accumulator --- */
            float i_active_thr = 0.08f * i_max;
            if (i_active_thr < 0.80f) i_active_thr = 0.80f;
            uint8_t has_active_current = (i_mag >= i_active_thr || s_app >= 5.0f);
            uint8_t has_stall_current = (i_mag >= i_stall_thr || s_app >= s_floor);
            uint8_t is_motoring = (g_foc.data.omega_elec * g_foc.data.Iq > 0.0f);

            /* Condition A: Observer Hallucination & High-Speed Desynchronization */
            /* Observer reports high RPM, but physical Back-EMF is absent (R_bemf < 0.20).
             * Gated only on active current, never gated on eta_em to avoid decoupling power
             * artifacts */
            uint8_t is_hallucination = has_active_current &&
                                       (speed_elec_abs > 1.2f * omega_stall_min_elec) &&
                                       (r_bemf < 0.20f);

            /* Condition B: Sudden Mechanical Jam / Hard Stall */
            uint8_t is_hard_jam =
                has_stall_current && is_motoring && (eta_em < 0.15f) &&
                (d_desync > 0.40f || r_bemf < 0.30f || speed_elec_abs < omega_stall_min_elec);

            /* Condition C: Low-Speed Overload Stall (disabled when regenerative braking) */
            uint8_t is_overload_stall = has_stall_current && is_motoring &&
                                        (speed_elec_abs < omega_stall_min_elec) && (eta_em < 0.20f);

            float risk_rate;
            if (is_hallucination || is_hard_jam) {
                risk_rate = 0.035f; /* Catastrophic trip in ~28-35 ms */
            } else if (is_overload_stall) {
                risk_rate = 0.020f; /* Progressive overload trip in ~50 ms */
            } else {
                risk_rate = -0.015f; /* Healthy operation: leak back to zero */
            }

            s_stall_risk_accum += risk_rate;
            if (s_stall_risk_accum > 1.0f) s_stall_risk_accum = 1.0f;
            if (s_stall_risk_accum < 0.0f) s_stall_risk_accum = 0.0f;
            g_foc.data.stall_risk = s_stall_risk_accum;

            if (s_stall_risk_accum >= 1.0f) {
                FOC_EnableDrivers(0);
                g_foc.status.fault = FOC_FAULT_STALL;
                g_foc.status.state = FOC_STATE_FAULT;
                return;
            }
        } else {
            FOC_ResetStallDetector();
        }

        /* 4. Continuous Current Saturation / Thermal Overload Protection (Layer 3) */
        if (fabsf(g_foc.data.Iq) >= 0.90f * g_foc.cfg.motor_max_curr) {
            current_sat_counter++;
            if (current_sat_counter >= 3000) { /* 3.0 seconds at >90% max current */
                FOC_EnableDrivers(0);
                g_foc.status.fault = FOC_FAULT_STALL;
                g_foc.status.state = FOC_STATE_FAULT;
                return;
            }
        } else {
            if (current_sat_counter > 0) current_sat_counter--;
        }
    } else {
        ground_fault_count = 0;
        stall_counter = 0;
        current_sat_counter = 0;
        FOC_ResetStallDetector();
    }
}

void FOC_SlowTask(void) {
    if (FOC_GetState() == FOC_STATE_IDLE && !FOC_IsInitialized()) return;
    if (!FOC_IsInitialized()) return;

    FOC_Input_Update();
    uint16_t vbus_raw = adc_regular_buffer[2];
    if (vbus_raw != 0) {
        float vref = ADC_Vref;
        if (vref < 1.0f) vref = 3.3f;
        float v_scale = vref * (VBUS_DIVIDER_RATIO / 4096.0f);
        float vbus_new = (float)vbus_raw * v_scale;

        g_foc.data.Vbus = VBUS_IIR_ALPHA * g_foc.data.Vbus + (1.0f - VBUS_IIR_ALPHA) * vbus_new;
        if (g_foc.data.Vbus < 1.0f) g_foc.data.Vbus = 1.0f;
        g_foc.data.Vbus_inv = 1.0f / g_foc.data.Vbus;

        float i_th_noise = g_foc.noise_profile.noise_pk_pk * 1.5f;
        float i_th_ripple = g_foc.data.Vbus / (4.0f * g_foc.cfg.pwm_frequency * g_foc.cfg.motor_ls);
        float i_th_min = g_foc.cfg.motor_max_curr * 0.03f;
        float i_th = i_th_noise;
        if (i_th_ripple > i_th) i_th = i_th_ripple;
        if (i_th_min > i_th) i_th = i_th_min;
        if (i_th < 0.050f) i_th = 0.050f;
        g_foc.data.inv_i_th = 1.0f / i_th;
    }

    if (g_foc.status.state == FOC_STATE_RUN || g_foc.status.state == FOC_STATE_STARTUP ||
        g_foc.status.state == FOC_STATE_ALIGN) {
        float p_elec = 1.5f * (g_foc.data.Vd * g_foc.data.Id + g_foc.data.Vq * g_foc.data.Iq);
        float ibus_raw = (g_foc.data.Vbus > 1.0f) ? (p_elec * g_foc.data.Vbus_inv) : 0.0f;
        g_foc.data.Ibus = g_foc.data.Ibus + 0.01f * (ibus_raw - g_foc.data.Ibus);
    } else {
        g_foc.data.Ibus = 0.0f;
    }
    FOC_Safety();

    if (g_foc.status.state == FOC_STATE_RUN) {
        MotorID_InertiaSlowTask(); /* Offline inertia measurement hook */

        if (g_foc.status.control_mode == FOC_MODE_SPEED) {
            if (!g_foc.status.in_transition) {
                float accel_rate = g_foc.cfg.speed_ramp_accel * 0.001f;
                float decel_rate = g_foc.cfg.speed_ramp_decel * 0.001f;
                float ramp_error = g_foc.cmd.speed_ref_target - g_foc.cmd.speed_ref;

                if (ramp_error > accel_rate) {
                    g_foc.cmd.speed_ref += accel_rate;
                } else if (ramp_error < -decel_rate) {
                    g_foc.cmd.speed_ref -= decel_rate;
                } else {
                    g_foc.cmd.speed_ref = g_foc.cmd.speed_ref_target;
                }
            } else {
                /* Hold speed reference steady at handoff speed during 20ms angle & Id blending */
                g_foc.cmd.speed_ref = g_foc.startup.omega;
            }

            float target_iq =
                LADRC_Update(&g_foc.ctrl.speed, g_foc.cmd.speed_ref, g_foc.data.omega_elec);

            g_foc.cmd.Iq_ref = target_iq;
            LADRC_SetActualOutput(&g_foc.ctrl.speed, g_foc.cmd.Iq_ref);
        }
    }

    if (g_foc.status.state == FOC_STATE_FAULT || g_foc.status.control_mode != FOC_MODE_VOLTAGE) {
        g_foc.cmd.Vq_ref = 0.0f;
        g_foc.cmd.Vq_ref_target = 0.0f;
    }
    if (g_foc.status.state == FOC_STATE_IDLE) {
        g_foc.cmd.Vq_ref = 0.0f;
    }
}
