/**
 * @file foc_calibration.c
 * @brief FOC ADC Offset Calibration, Noise Characterization & AWD Protection Implementation
 */

#include "foc_calibration.h"

#include <math.h>

#include "flash_config.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "motor_id.h"
#include "peripheral_init.h"

/* Phase voltage calibration accumulators */
static int32_t s_vphase_a_accum = 0;
static int32_t s_vphase_c_accum = 0;

static uint16_t s_vphase_a_min = 4096;
static uint16_t s_vphase_a_max = 0;
static uint16_t s_vphase_c_min = 4096;
static uint16_t s_vphase_c_max = 0;

/* Current sensing calibration accumulators */
static uint32_t s_adc1_accum = 0;
static uint32_t s_adc2_b_accum = 0;
static uint32_t s_adc1_c_pb1_accum = 0;
static uint32_t s_adc2_c_opamp3_accum = 0;
static uint16_t s_cal_samples = 0;
static uint8_t s_cal_step = 0;
static float s_cal_avg_var = 0.0f;

/* Current sensing noise characterization accumulators */
static uint64_t s_adc1_sq_accum = 0;
static uint64_t s_adc2_b_sq_accum = 0;
static uint64_t s_adc1_c_pb1_sq_accum = 0;
static uint64_t s_adc2_c_opamp3_sq_accum = 0;
static uint16_t s_adc1_min = 4096;
static uint16_t s_adc1_max = 0;
static uint16_t s_adc2_b_min = 4096;
static uint16_t s_adc2_b_max = 0;
static uint16_t s_adc1_c_pb1_min = 4096;
static uint16_t s_adc1_c_pb1_max = 0;
static uint16_t s_adc2_c_opamp3_min = 4096;
static uint16_t s_adc2_c_opamp3_max = 0;

extern volatile float ADC_Vref;

void FOC_Calibration_Reset(void) {
    s_adc1_accum = 0;
    s_adc2_b_accum = 0;
    s_adc1_c_pb1_accum = 0;
    s_adc2_c_opamp3_accum = 0;
    s_cal_samples = 0;
    s_cal_step = 0;
    s_cal_avg_var = 0.0f;

    s_vphase_a_accum = 0;
    s_vphase_c_accum = 0;
    s_vphase_a_min = 4096;
    s_vphase_a_max = 0;
    s_vphase_c_min = 4096;
    s_vphase_c_max = 0;

    s_adc1_sq_accum = 0;
    s_adc2_b_sq_accum = 0;
    s_adc1_c_pb1_sq_accum = 0;
    s_adc2_c_opamp3_sq_accum = 0;
    s_adc1_min = 4096;
    s_adc1_max = 0;
    s_adc2_b_min = 4096;
    s_adc2_b_max = 0;
    s_adc1_c_pb1_min = 4096;
    s_adc1_c_pb1_max = 0;
    s_adc2_c_opamp3_min = 4096;
    s_adc2_c_opamp3_max = 0;
}

void FOC_Calibration_Accumulate(uint16_t adc1_data, uint16_t adc2_data, uint8_t cal_phase,
                                uint16_t reg_va, uint16_t reg_vc) {
    if (s_cal_samples >= CAL_SAMPLES) {
        return;
    }

    switch (cal_phase) {
        case 0: /* Skip A: ADC1 measured Phase C (PB1), ADC2 measured Phase B (VOPAMP2) */
            s_adc1_c_pb1_accum += adc1_data;
            s_adc1_c_pb1_sq_accum += (uint64_t)adc1_data * adc1_data;
            if (adc1_data < s_adc1_c_pb1_min) s_adc1_c_pb1_min = adc1_data;
            if (adc1_data > s_adc1_c_pb1_max) s_adc1_c_pb1_max = adc1_data;

            s_adc2_b_accum += adc2_data;
            s_adc2_b_sq_accum += (uint64_t)adc2_data * adc2_data;
            if (adc2_data < s_adc2_b_min) s_adc2_b_min = adc2_data;
            if (adc2_data > s_adc2_b_max) s_adc2_b_max = adc2_data;
            break;

        case 1: /* Skip B: ADC1 measured Phase A (VOPAMP1), ADC2 measured Phase C (VOPAMP3) */
            s_adc1_accum += adc1_data;
            s_adc1_sq_accum += (uint64_t)adc1_data * adc1_data;
            if (adc1_data < s_adc1_min) s_adc1_min = adc1_data;
            if (adc1_data > s_adc1_max) s_adc1_max = adc1_data;

            s_adc2_c_opamp3_accum += adc2_data;
            s_adc2_c_opamp3_sq_accum += (uint64_t)adc2_data * adc2_data;
            if (adc2_data < s_adc2_c_opamp3_min) s_adc2_c_opamp3_min = adc2_data;
            if (adc2_data > s_adc2_c_opamp3_max) s_adc2_c_opamp3_max = adc2_data;
            break;

        default: /* Skip C: ADC1 measured Phase A (VOPAMP1), ADC2 measured Phase B (VOPAMP2) */
            s_adc1_accum += adc1_data;
            s_adc1_sq_accum += (uint64_t)adc1_data * adc1_data;
            if (adc1_data < s_adc1_min) s_adc1_min = adc1_data;
            if (adc1_data > s_adc1_max) s_adc1_max = adc1_data;

            s_adc2_b_accum += adc2_data;
            s_adc2_b_sq_accum += (uint64_t)adc2_data * adc2_data;
            if (adc2_data < s_adc2_b_min) s_adc2_b_min = adc2_data;
            if (adc2_data > s_adc2_b_max) s_adc2_b_max = adc2_data;
            break;
    }

    {
        s_vphase_a_accum += reg_va;
        s_vphase_c_accum += reg_vc;
        if (reg_va < s_vphase_a_min) s_vphase_a_min = reg_va;
        if (reg_va > s_vphase_a_max) s_vphase_a_max = reg_va;
        if (reg_vc < s_vphase_c_min) s_vphase_c_min = reg_vc;
        if (reg_vc > s_vphase_c_max) s_vphase_c_max = reg_vc;
    }
    s_cal_samples++;
}

void FOC_ConfigureAWD(void) {
    float oc_thr = FOC_GetOCThreshold();
    if (oc_thr <= 0.001f) {
        LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, 4095, 0);
        LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD2, 4095, 0);
        LL_ADC_ConfigAnalogWDThresholds(ADC2, LL_ADC_AWD1, 4095, 0);

        LL_ADC_DisableIT_AWD1(ADC1);
        LL_ADC_DisableIT_AWD2(ADC1);
        LL_ADC_DisableIT_AWD1(ADC2);
        return;
    }

    float vref = ADC_Vref;
    if (vref < 1.0f) vref = 3.3f;

    /* 1. Resolve the overcurrent threshold in LSB counts */
    const float gain = ADC_RESOLUTION * OPAMP_GAIN * SHUNT_RESISTANCE;
    float current_to_adc = gain / vref;
    int32_t adc_step = (int32_t)(oc_thr * current_to_adc + 0.5f);

    int32_t hw_max_ac = g_foc.cfg.cal_offset_a;
    int32_t headroom = 4095 - g_foc.cfg.cal_offset_a;
    if (headroom < hw_max_ac) hw_max_ac = headroom;

    int32_t lim_c_pb1 = g_foc.cfg.cal_offset_c_pb1;
    headroom = 4095 - g_foc.cfg.cal_offset_c_pb1;
    if (headroom < lim_c_pb1) lim_c_pb1 = headroom;
    if (lim_c_pb1 < hw_max_ac) hw_max_ac = lim_c_pb1;

    int32_t hw_max_bc = g_foc.cfg.cal_offset_b;
    headroom = 4095 - g_foc.cfg.cal_offset_b;
    if (headroom < hw_max_bc) hw_max_bc = headroom;
    int32_t lim_c_op3 = g_foc.cfg.cal_offset_c_op3;

    headroom = 4095 - g_foc.cfg.cal_offset_c_op3;
    if (headroom < lim_c_op3) lim_c_op3 = headroom;
    if (lim_c_op3 < hw_max_bc) hw_max_bc = lim_c_op3;

    if (adc_step < 1 || adc_step > (int32_t)(0.95f * (float)hw_max_ac) ||
        adc_step > (int32_t)(0.95f * (float)hw_max_bc)) {
        LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, 4095, 0);
        LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD2, 4095, 0);
        LL_ADC_ConfigAnalogWDThresholds(ADC2, LL_ADC_AWD1, 4095, 0);

        LL_ADC_DisableIT_AWD1(ADC1);
        LL_ADC_DisableIT_AWD2(ADC1);
        LL_ADC_DisableIT_AWD1(ADC2);
        return;
    }

    /* Phase A and Phase C (ADC1 AWD1 - monitors all injected channels: VOPAMP1 and CH12) */
    int32_t avg_offset_ac = (g_foc.cfg.cal_offset_a + g_foc.cfg.cal_offset_c_pb1) / 2;
    int32_t high_ac = avg_offset_ac + adc_step;
    int32_t low_ac = avg_offset_ac - adc_step;
    if (high_ac > 4095) high_ac = 4095;
    if (low_ac < 0) low_ac = 0;

    /* Phase B and C (ADC2 AWD1 - monitors all injected channels: VOPAMP2 and VOPAMP3) */
    int32_t avg_offset_bc = (g_foc.cfg.cal_offset_b + g_foc.cfg.cal_offset_c_op3) / 2;
    int32_t high_bc = avg_offset_bc + adc_step;
    int32_t low_bc = avg_offset_bc - adc_step;
    if (high_bc > 4095) high_bc = 4095;
    if (low_bc < 0) low_bc = 0;

    /* ADC1: AWD1 for all Injected Channels (Phase A and Phase C via PB1) */
    LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, high_ac, low_ac);
    LL_ADC_SetAWDFilteringConfiguration(ADC1, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);

    /* ADC2: AWD1 for all Injected Channels (Phase B and Phase C via VOPAMP3) */
    LL_ADC_SetAnalogWDMonitChannels(ADC2, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC2, LL_ADC_AWD1, high_bc, low_bc);
    LL_ADC_SetAWDFilteringConfiguration(ADC2, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);

    /* Clear all pending AWD flags first to prevent stale interrupt triggers */
    LL_ADC_ClearFlag_AWD1(ADC1);
    LL_ADC_ClearFlag_AWD1(ADC2);

    LL_ADC_EnableIT_AWD1(ADC1);
    LL_ADC_EnableIT_AWD1(ADC2);
}

void FOC_StateCalibration(void) {
    if (s_cal_samples >= CAL_SAMPLES) {
        const float inv_n_ab = 1.0f / (CAL_SAMPLES * (2.0f / 3.0f));
        const float inv_n_c = 1.0f / (CAL_SAMPLES / 3.0f);

        if (s_cal_step == 0) {
            /* Step 0 (Tick 768): Compute Offsets and Variances (Pure FPU: ~25 cycles) */
            float mean_a = (float)s_adc1_accum * inv_n_ab;
            float mean_b = (float)s_adc2_b_accum * inv_n_ab;
            float mean_c_pb1 = (float)s_adc1_c_pb1_accum * inv_n_c;
            float mean_c_op3 = (float)s_adc2_c_opamp3_accum * inv_n_c;

            g_foc.cfg.cal_offset_a = (int32_t)(mean_a + 0.5f);
            g_foc.cfg.cal_offset_b = (int32_t)(mean_b + 0.5f);
            g_foc.cfg.cal_offset_c_pb1 = (int32_t)(mean_c_pb1 + 0.5f);
            g_foc.cfg.cal_offset_c_op3 = (int32_t)(mean_c_op3 + 0.5f);
            g_foc.cfg.cal_valid = 1;

            float var_a = ((float)s_adc1_sq_accum * inv_n_ab) - (mean_a * mean_a);
            float var_b = ((float)s_adc2_b_sq_accum * inv_n_ab) - (mean_b * mean_b);
            float var_c_pb1 = ((float)s_adc1_c_pb1_sq_accum * inv_n_c) - (mean_c_pb1 * mean_c_pb1);
            float var_c_op3 =
                ((float)s_adc2_c_opamp3_sq_accum * inv_n_c) - (mean_c_op3 * mean_c_op3);
            if (var_a < 0.0f) var_a = 0.0f;
            if (var_b < 0.0f) var_b = 0.0f;
            if (var_c_pb1 < 0.0f) var_c_pb1 = 0.0f;
            if (var_c_op3 < 0.0f) var_c_op3 = 0.0f;

            s_cal_avg_var = (var_a + var_b + 0.5f * (var_c_pb1 + var_c_op3)) * (1.0f / 3.0f);

            s_cal_step = 1;
            return;
        }

        /* Step 1 (Tick 769): Noise Characterization, AWD Hardware Setup & Transition */
        float i_scale = ADC_Vref * fabsf(ADC_TO_CURRENT);
        float noise_rms = sqrtf(s_cal_avg_var) * i_scale;

        int32_t pk_a = (int32_t)s_adc1_max - (int32_t)s_adc1_min;
        int32_t pk_b = (int32_t)s_adc2_b_max - (int32_t)s_adc2_b_min;
        int32_t pk_c1 = (int32_t)s_adc1_c_pb1_max - (int32_t)s_adc1_c_pb1_min;
        int32_t pk_c2 = (int32_t)s_adc2_c_opamp3_max - (int32_t)s_adc2_c_opamp3_min;
        int32_t max_pk = (pk_a > pk_b) ? pk_a : pk_b;
        if (pk_c1 > max_pk) max_pk = pk_c1;
        if (pk_c2 > max_pk) max_pk = pk_c2;
        float noise_pk_pk = (float)max_pk * i_scale;

        g_foc.cfg.cal_noise_rms = noise_rms;
        g_foc.cfg.cal_noise_pk_pk = noise_pk_pk;
        g_foc.cfg.cal_is_flat_thr = clampf(0.5f * noise_pk_pk, 0.003f, 0.500f);
        g_foc.cfg.cal_i_inj_min = clampf(3.0f * noise_rms, 0.020f, 1.0f);

        /* Restore hardware sensing to Phase C skip */
        FOC_HW_SwitchCurrentSensing(2);

        /* Check spinning detection */
        uint16_t diff_a = s_vphase_a_max - s_vphase_a_min;
        uint16_t diff_c = s_vphase_c_max - s_vphase_c_min;
        uint8_t is_spinning = (diff_a > 30) || (diff_c > 30);
        if (!is_spinning) {
            const float inv_cal_samples = 1.0f / (float)CAL_SAMPLES;
            g_foc.cfg.cal_vphase_a = (int32_t)((float)s_vphase_a_accum * inv_cal_samples);
            g_foc.cfg.cal_vphase_c = (int32_t)((float)s_vphase_c_accum * inv_cal_samples);
        }

        /* Synchronize to Flash config */
        FlashConfig_t* cfg_stored = FlashConfig_Get();
        cfg_stored->cal_offset_a = g_foc.cfg.cal_offset_a;
        cfg_stored->cal_offset_b = g_foc.cfg.cal_offset_b;
        cfg_stored->cal_offset_c_pb1 = g_foc.cfg.cal_offset_c_pb1;
        cfg_stored->cal_offset_c_op3 = g_foc.cfg.cal_offset_c_op3;
        cfg_stored->cal_noise_rms = g_foc.cfg.cal_noise_rms;
        cfg_stored->cal_noise_pk_pk = g_foc.cfg.cal_noise_pk_pk;
        cfg_stored->cal_is_flat_thr = g_foc.cfg.cal_is_flat_thr;
        cfg_stored->cal_i_inj_min = g_foc.cfg.cal_i_inj_min;
        if (!is_spinning) {
            cfg_stored->cal_vphase_a = g_foc.cfg.cal_vphase_a;
            cfg_stored->cal_vphase_c = g_foc.cfg.cal_vphase_c;
        }
        cfg_stored->cal_valid = 1;

        /* Configure hardware watchdog now that offsets are confirmed */
        FOC_ConfigureAWD();

        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        g_foc.startup.counter = 0;
        s_cal_step = 0;

        g_foc.cfg.motor_min_spd = FOC_CalculateObserverMinSpeed();

        if (id_result.state != MOTOR_ID_STATE_ALIGN) {
            g_foc.status.state = FOC_STATE_DETECT;
        } else {
            FOC_EnableDrivers(1);
            MotorID_Start();
            g_foc.status.state = FOC_STATE_SELF_COMMISSION;
        }
    }
}

float FOC_CalculateObserverMinSpeed(void) {
    /* 1. In-Band noise attenuation using PLL minimum cutoff bandwidth:
     * Directly links to smo->pll_cutoff_min without any complex math! */
    float pll_bw = g_foc.ctrl.smo.pll_cutoff_min;
    if (pll_bw < 10.0f) pll_bw = 10.0f; /* Safety baseline */
    float f_nyquist = g_foc.cfg.pwm_frequency * 0.5f;
    float k_atten = sqrtf(pll_bw / f_nyquist);
    float noise_in_band = g_foc.cfg.cal_noise_rms * k_atten;
    float v_current_noise = noise_in_band * g_foc.cfg.motor_rs;
    /* 2. Effective residual deadtime voltage after active DTC (residual ~5%) */
    float v_deadtime = g_foc.deadtime_duty * g_foc.data.Vbus;
    float v_dt_residual = 0.05f * v_deadtime;
    /* 3. Physical hardware floor (~2 LSB ADC quantization residual) */
    const float v_hw_floor = 0.025f;
    /* 4. Total In-Band Noise Floor */
    float v_noise_floor = v_current_noise + v_dt_residual + v_hw_floor;
    /* 5. 3-Sigma Target BEMF (99.73% statistical confidence) */
    const float k_snr = 3.0f;
    float e_bemf_target = k_snr * v_noise_floor;
    if (e_bemf_target < 0.080f) {
        e_bemf_target = 0.080f;
    }
    /* 6. Convert target BEMF directly to electrical angular velocity [rad/s elec] */
    float rec_handoff_omega = 0.0f;
    if (g_foc.cfg.motor_flux > 1e-6f) {
        rec_handoff_omega = e_bemf_target / g_foc.cfg.motor_flux;
    } else if (g_foc.cfg.motor_kv > 1.0f && g_foc.cfg.motor_poles >= 1.0f) {
        float flux_fallback =
            60.0f / (1.732f * g_foc.cfg.motor_kv * TWO_PI * (float)g_foc.cfg.motor_poles);
        rec_handoff_omega = (flux_fallback > 1e-6f) ? (e_bemf_target / flux_fallback) : 700.0f;
    } else {
        rec_handoff_omega = 700.0f;
    }
    /* 7. Safety constraints:
     * - Minimum 15 Hz electrical frequency for clean STF/PLL tracking (~94.25 rad/s elec)
     * - Maximum clamp to 25% of max rated speed */
    float min_elec_omega = 15.0f * TWO_PI;
    if (rec_handoff_omega < min_elec_omega) {
        rec_handoff_omega = min_elec_omega;
    }
    if (g_foc.cfg.motor_max_spd > 0.0f) {
        float max_handoff_limit = 0.25f * g_foc.cfg.motor_max_spd;
        if (rec_handoff_omega > max_handoff_limit) {
            rec_handoff_omega = max_handoff_limit;
        }
    }
    return rec_handoff_omega;
}
