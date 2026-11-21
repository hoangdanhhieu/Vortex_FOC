/**
 * @file foc_slow_task.c
 * @brief FOC 1kHz Slow Task, Speed Loop, Ramps & Safety Protections Implementation
 */

#include "foc_slow_task.h"

#include <math.h>

#include "foc_config.h"
#include "foc_hardware.h"
#include "foc_input.h"
#include "foc_state_machine.h"
#include "ladrc_controller.h"
#include "motor_id.h"
#include "peripheral_init.h"

static uint32_t current_sat_counter = 0; /* Sustained current saturation overload timer */
static uint32_t ov_count = 0;            /* OV debounce counter (1 ms ticks) */
static uint32_t uv_count = 0;            /* UV debounce counter (1 ms ticks) */
static uint32_t overspeed_count = 0;     /* Overspeed/NaN debounce counter (1 ms ticks) */

/* Scale-invariant physical stall & desync detector states */
static float s_stall_risk_accum = 0.0f; /* Leaky stall risk accumulator [0.0 to 1.0] */
static float s_bemf_peak = 0.0f;        /* 1kHz online BEMF envelope peak tracker [V] */
static float s_bemf_valley = 0.0f;      /* 1kHz online BEMF envelope valley tracker [V] */

extern volatile float ADC_Vref;

void FOC_ResetStallDetector(void) {
    s_stall_risk_accum = 0.0f;
    s_bemf_peak = 0.0f;
    s_bemf_valley = 0.0f;
    g_foc.data.stall_risk = 0.0f;
    g_foc.data.eta_em = 10.0f;
    g_foc.data.d_desync = 0.0f;
    g_foc.data.r_bemf = 1.0f;
}

void FOC_Safety() {
    if (g_foc.status.state == FOC_STATE_RUN || g_foc.status.state == FOC_STATE_STARTUP ||
        g_foc.status.state == FOC_STATE_ALIGN) {
        if (g_foc.data.Vbus > g_foc.cfg.fault_ov_threshold) {
            if (ov_count < 10) ov_count++;
            if (ov_count >= 10) {
                FOC_EnableDrivers(0);
                g_foc.status.fault = FOC_FAULT_OVERVOLTAGE;
                g_foc.status.state = FOC_STATE_FAULT;
            }
        } else if (ov_count > 0) {
            ov_count--;
        }
        if (g_foc.data.Vbus < g_foc.cfg.fault_uv_threshold) {
            if (uv_count < 10) uv_count++;
            if (uv_count >= 10) {
                FOC_EnableDrivers(0);
                g_foc.status.fault = FOC_FAULT_UNDERVOLTAGE;
                g_foc.status.state = FOC_STATE_FAULT;
            }
        } else if (uv_count > 0) {
            uv_count--;
        }
    } else {
        ov_count = 0;
        uv_count = 0;
    }

    /* Motor Identification 1kHz Slow Task */
    if (g_foc.status.state == FOC_STATE_SELF_COMMISSION) {
        MotorID_SlowTask();
    }

    if (g_foc.status.state == FOC_STATE_RUN) {
        uint8_t reversal_grace = FOC_IsReversalGraceActive();

        /* 1. Observer Integrity & Overspeed Protection (NaN/Inf + 5 ms debounce) */
        if (!reversal_grace) {
            float omega_abs = fabsf(g_foc.data.omega_elec);
            if (!isfinite(omega_abs) ||
                (g_foc.cfg.motor_max_spd > 0.0f && omega_abs > g_foc.cfg.motor_max_spd * 1.25f)) {
                if (overspeed_count < 10) overspeed_count++;
                if (overspeed_count >= 10) {
                    FOC_EnableDrivers(0);
                    g_foc.status.fault = FOC_FAULT_OBSERVER_FAIL;
                    g_foc.status.state = FOC_STATE_FAULT;
                    return;
                }
            } else if (overspeed_count > 0) {
                overspeed_count--;
            }
        } else {
            overspeed_count = 0;
        }

        /* 2. Advanced 3-Pillar & Scale-Invariant Stall/Desync Protection (Hardware Noise Adaptive)
         */
        if (g_foc.cfg.fault_stall_enable && !reversal_grace) {
            float v_d = g_foc.data.Vd;
            float v_q = g_foc.data.Vq;
            float i_d = g_foc.data.Id;
            float i_q = g_foc.data.Iq;
            float i_mag = sqrtf(i_d * i_d + i_q * i_q);
            float rs = g_foc.cfg.motor_rs;
            float flux = g_foc.cfg.motor_flux;
            float omega_est = g_foc.data.omega_elec;
            float speed_abs = fabsf(omega_est);
            float handoff_speed = g_foc.cfg.startup_handoff_speed;

            /* Active current floor: dynamically adapts to calibrated peak-to-peak hardware noise */
            float active_current_floor = fmaxf(2.0f * g_foc.cfg.cal_noise_pk_pk, 0.50f);
            uint8_t has_active_current = (i_mag >= active_current_floor);

            /* --- 1. Scale-Invariant Dimensionless Metrics Calculation --- */
            /* Power Ratio: P_in / P_loss_cu (Conservation of Energy - universally applied across
             * all modes) */
            float p_in = 1.5f * (v_d * i_d + v_q * i_q);
            float p_cu_cfg = 1.5f * rs * (i_mag * i_mag);
            float power_ratio = (p_cu_cfg > 0.05f) ? (p_in / p_cu_cfg) : 10.0f;

            /* d-axis cross-coupling leakage ratio: |Id| / I_mag */
            float id_ratio = (i_mag > 0.10f) ? (fabsf(i_d) / i_mag) : 0.0f;

            /* BEMF tracking ratio: BEMF_mag / (omega_est * flux) */
            float e_obs_mag = g_foc.ctrl.smo.bemf_mag;
            float e_expected = speed_abs * flux;
            float bemf_ratio = (e_expected > 0.10f) ? (e_obs_mag / e_expected) : 1.0f;

            /* 1 kHz online BEMF peak/valley envelope tracker for cyclical ripple estimation */
            if (e_obs_mag > s_bemf_peak) {
                s_bemf_peak = e_obs_mag;
            } else {
                s_bemf_peak -= 0.03f * (s_bemf_peak - e_obs_mag);
            }
            if (e_obs_mag < s_bemf_valley) {
                s_bemf_valley = e_obs_mag;
            } else {
                s_bemf_valley += 0.03f * (e_obs_mag - s_bemf_valley);
            }
            float ripple_cyclic =
                (s_bemf_peak > 0.05f) ? ((s_bemf_peak - s_bemf_valley) / s_bemf_peak) : 0.0f;

            /* --- Real-Time Telemetry Logging --- */
            g_foc.data.eta_em = power_ratio;
            g_foc.data.d_desync = id_ratio;
            g_foc.data.r_bemf = bemf_ratio;
            g_foc.data.stall_risk = s_stall_risk_accum;

            /* --- 2. 4-Quadrant Smart Regenerative Braking Lockout --- */
            float dir_sign = g_foc.status.reverse;
            uint8_t is_braking = 0;

            if (g_foc.status.control_mode == FOC_MODE_VOLTAGE) {
                uint8_t is_braking_phys = ((omega_est * dir_sign) > handoff_speed) &&
                                          ((i_q * dir_sign) < -active_current_floor);
                float vq_tgt = g_foc.cmd.Vq_ref_target;
                uint8_t rev_req = (fabsf(vq_tgt) > 1e-3f) && ((vq_tgt * dir_sign) < 0.0f);
                uint8_t zero_stop =
                    (fabsf(vq_tgt) <= 1e-3f) && ((i_q * dir_sign) < -active_current_floor);
                is_braking = is_braking_phys || rev_req || zero_stop;
            } else if (g_foc.status.control_mode == FOC_MODE_SPEED) {
                uint8_t zero_spd_cmd = (fabsf(g_foc.cmd.speed_ref) <= 1.0f) &&
                                       (fabsf(g_foc.cmd.speed_ref_target) <= 1.0f);
                is_braking = ((g_foc.cmd.Iq_ref * dir_sign) < -0.05f) || zero_spd_cmd;
            } else { /* FOC_MODE_TORQUE */
                is_braking = ((g_foc.cmd.Iq_ref * dir_sign) < -0.05f);
            }

            uint8_t fault_candidate = 0;
            uint8_t is_catastrophic = 0;

            /* Branch 0: Numerical explosion / Observer clamping runaway (Physical speed ceiling
             * exceeded) */
            if (speed_abs > 1.35f * g_foc.cfg.motor_max_spd) {
                fault_candidate = 1;
                is_catastrophic = 1;
            } else if (!is_braking) {
                /* Branch 1: Mechanical power collapse / Rotor stall / Observer hallucination
                 * (Conservation of Energy) */
                if (power_ratio <= 1.85f && has_active_current) {
                    fault_candidate = 1;
                    is_catastrophic = 1;
                }
                /* Branch 2: Kinematic pole slip / Forced reverse rotation (Rotor jerked backward by
                   external load) */
                else if ((omega_est * dir_sign) < -100.0f && has_active_current) {
                    fault_candidate = 1;
                    is_catastrophic = 1;
                }
                /* Branch 3: Broken phase (A/B/C disconnect) or severe elliptical BEMF distortion */
                else if (ripple_cyclic > 0.25f && id_ratio > 0.25f && has_active_current) {
                    fault_candidate = 1;
                    is_catastrophic = 1;
                }
                /* Branch 4: Progressive overload pulling down rotor speed */
                else if (power_ratio <= 1.95f && has_active_current) {
                    fault_candidate = 1;
                }
            }

            float risk_rate;
            if (fault_candidate) {
                if (is_catastrophic) {
                    risk_rate = 0.030f;
                } else {
                    risk_rate = 0.015f;
                }
            } else {
                risk_rate = -0.020f;
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

        /* 4. Continuous Current Saturation / Thermal Overload Protection */
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
        current_sat_counter = 0;
        overspeed_count = 0;
        FOC_ResetStallDetector();
    }
}

void FOC_SlowTask(void) {
    if (!FOC_IsInitialized()) return;

    FOC_Input_Update();
    uint16_t vbus_raw = adc_regular_buffer[2];
    if (vbus_raw != 0) {
        float vref = ADC_Vref;
        if (vref < 1.0f) vref = 3.3f;
        float v_scale = vref * (VBUS_DIVIDER_RATIO / 4096.0f);
        float vbus_new = (float)vbus_raw * v_scale;

        float vbus = VBUS_IIR_ALPHA * g_foc.data.Vbus + (1.0f - VBUS_IIR_ALPHA) * vbus_new;
        if (vbus < 1.0f) vbus = 1.0f;
        float vbus_inv = 1.0f / vbus;
        float v_max = vbus * g_foc.v_max_scale;
        float i_th_noise = g_foc.cfg.cal_noise_pk_pk * 3.0f;
        float i_th_min = g_foc.cfg.motor_max_curr * 0.02f;
        float i_th = (i_th_noise > i_th_min) ? i_th_noise : i_th_min;
        float inv_i_th = 1.0f / i_th;

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        g_foc.data.Vbus = vbus;
        g_foc.data.Vbus_inv = vbus_inv;
        g_foc.v_max = v_max;
        PI_SetLimits(&g_foc.ctrl.id, -v_max, v_max);
        PI_SetIntLimits(&g_foc.ctrl.id, -v_max, v_max);
        g_foc.data.inv_i_th = inv_i_th;
        if (!primask) {
            __enable_irq();
        }
    }

    if (g_foc.status.state == FOC_STATE_RUN || g_foc.status.state == FOC_STATE_STARTUP ||
        g_foc.status.state == FOC_STATE_ALIGN) {
        float p_elec = 1.5f * (g_foc.data.Vd * g_foc.data.Id + g_foc.data.Vq * g_foc.data.Iq);
        float ibus_raw = (g_foc.data.Vbus > 1.0f) ? (p_elec * g_foc.data.Vbus_inv) : 0.0f;
        g_foc.data.Ibus = g_foc.data.Ibus + 0.02f * (ibus_raw - g_foc.data.Ibus);
    } else {
        g_foc.data.Ibus = 0.0f;
    }
    FOC_Safety();

    if (g_foc.status.state == FOC_STATE_RUN) {
        MotorID_InertiaSlowTask();

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

            g_foc.cmd.Iq_ref_target = target_iq;
            LADRC_SetActualOutput(&g_foc.ctrl.speed, g_foc.cmd.Iq_ref_target);
        }
    }

    if (g_foc.status.state == FOC_STATE_FAULT || g_foc.status.control_mode != FOC_MODE_VOLTAGE) {
        g_foc.cmd.Vq_ref = 0.0f;
        g_foc.cmd.Vq_ref_target = 0.0f;
    }
    if (g_foc.status.state == FOC_STATE_IDLE) {
        g_foc.cmd.Vq_ref = 0.0f;
    }

    FOC_State_t cur_state = g_foc.status.state;
    uint8_t is_obs_active = (cur_state == FOC_STATE_RUN || cur_state == FOC_STATE_STARTUP ||
                             cur_state == FOC_STATE_FLYING_START || cur_state == FOC_STATE_ALIGN ||
                             cur_state == FOC_STATE_DETECT)
                                ? 1U
                                : 0U;
    SMO_SlowTask(&g_foc.ctrl.smo, is_obs_active, (cur_state == FOC_STATE_STARTUP) ? 1U : 0U,
                 g_foc.startup.omega, g_foc.data.Vbus);
}
