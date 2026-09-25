/**
 * @file foc_calibration.c
 * @brief FOC ADC Offset Calibration, Noise Characterization & AWD Protection Implementation
 */

#include "foc_calibration.h"

#include <math.h>

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

extern volatile uint16_t adc_regular_buffer[3];
extern volatile float ADC_Vref;

void FOC_Calibration_Reset(void) {
    g_foc.adc_cal.offset_a = 0;
    g_foc.adc_cal.offset_b = 0;
    g_foc.adc_cal.offset_c_pb1 = 0;
    g_foc.adc_cal.offset_c_opamp3 = 0;
    g_foc.adc_cal.cal_samples = 0;

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

void FOC_Calibration_Accumulate(uint16_t adc1_data, uint16_t adc2_data, uint8_t cal_phase) {
    switch (cal_phase) {
        case 0: /* Skip A: ADC1 measured Phase C (PB1), ADC2 measured Phase B (VOPAMP2) */
            g_foc.adc_cal.offset_c_pb1 += adc1_data;
            s_adc1_c_pb1_sq_accum += (uint64_t)adc1_data * adc1_data;
            if (adc1_data < s_adc1_c_pb1_min) s_adc1_c_pb1_min = adc1_data;
            if (adc1_data > s_adc1_c_pb1_max) s_adc1_c_pb1_max = adc1_data;

            g_foc.adc_cal.offset_b += adc2_data;
            s_adc2_b_sq_accum += (uint64_t)adc2_data * adc2_data;
            if (adc2_data < s_adc2_b_min) s_adc2_b_min = adc2_data;
            if (adc2_data > s_adc2_b_max) s_adc2_b_max = adc2_data;
            break;

        case 1: /* Skip B: ADC1 measured Phase A (VOPAMP1), ADC2 measured Phase C (VOPAMP3) */
            g_foc.adc_cal.offset_a += adc1_data;
            s_adc1_sq_accum += (uint64_t)adc1_data * adc1_data;
            if (adc1_data < s_adc1_min) s_adc1_min = adc1_data;
            if (adc1_data > s_adc1_max) s_adc1_max = adc1_data;

            g_foc.adc_cal.offset_c_opamp3 += adc2_data;
            s_adc2_c_opamp3_sq_accum += (uint64_t)adc2_data * adc2_data;
            if (adc2_data < s_adc2_c_opamp3_min) s_adc2_c_opamp3_min = adc2_data;
            if (adc2_data > s_adc2_c_opamp3_max) s_adc2_c_opamp3_max = adc2_data;
            break;

        default: /* Skip C: ADC1 measured Phase A (VOPAMP1), ADC2 measured Phase B (VOPAMP2) */
            g_foc.adc_cal.offset_a += adc1_data;
            s_adc1_sq_accum += (uint64_t)adc1_data * adc1_data;
            if (adc1_data < s_adc1_min) s_adc1_min = adc1_data;
            if (adc1_data > s_adc1_max) s_adc1_max = adc1_data;

            g_foc.adc_cal.offset_b += adc2_data;
            s_adc2_b_sq_accum += (uint64_t)adc2_data * adc2_data;
            if (adc2_data < s_adc2_b_min) s_adc2_b_min = adc2_data;
            if (adc2_data > s_adc2_b_max) s_adc2_b_max = adc2_data;
            break;
    }

    {
        uint16_t va = adc_regular_buffer[0];
        uint16_t vc = adc_regular_buffer[1];
        s_vphase_a_accum += va;
        s_vphase_c_accum += vc;
        if (va < s_vphase_a_min) s_vphase_a_min = va;
        if (va > s_vphase_a_max) s_vphase_a_max = va;
        if (vc < s_vphase_c_min) s_vphase_c_min = vc;
        if (vc > s_vphase_c_max) s_vphase_c_max = vc;
    }
    g_foc.adc_cal.cal_samples++;
}

void FOC_ConfigureAWD(void) {
    if (g_foc.adc_cal.offset_a < 200 || g_foc.adc_cal.offset_b < 200 ||
        g_foc.adc_cal.offset_c_pb1 < 200 || g_foc.adc_cal.offset_c_opamp3 < 200) {
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

    // 1. Calculate Overcurrent thresholds in LSB counts
    const float gain = ADC_RESOLUTION * OPAMP_GAIN * SHUNT_RESISTANCE;
    float current_to_adc = gain / vref;
    float adc_step = g_foc.cfg.fault_oc_threshold * current_to_adc;

    // Phase A and Phase C (ADC1 AWD1 - monitors all injected channels: VOPAMP1 and CH12)
    int32_t avg_offset_ac = (g_foc.adc_cal.offset_a + g_foc.adc_cal.offset_c_pb1) / 2;
    int32_t high_ac = avg_offset_ac + (int32_t)adc_step;
    int32_t low_ac = avg_offset_ac - (int32_t)adc_step;
    if (high_ac > 4095) high_ac = 4095;
    if (low_ac < 0) low_ac = 0;

    // Phase B and C (ADC2 AWD1 - monitors all injected channels: VOPAMP2 and VOPAMP3)
    int32_t avg_offset_bc = (g_foc.adc_cal.offset_b + g_foc.adc_cal.offset_c_opamp3) / 2;
    int32_t high_bc = avg_offset_bc + (int32_t)adc_step;
    int32_t low_bc = avg_offset_bc - (int32_t)adc_step;
    if (high_bc > 4095) high_bc = 4095;
    if (low_bc < 0) low_bc = 0;

    // 2. Vbus protection is now handled in software (FOC_SlowTask at 1kHz)

    // 3. Program hardware registers using LL driver
    // ADC1: AWD1 for all Injected Channels (Phase A and Phase C via PB1)
    LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, high_ac, low_ac);
    LL_ADC_SetAWDFilteringConfiguration(ADC1, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);

    // ADC2: AWD1 for all Injected Channels (Phase B and Phase C via VOPAMP3)
    LL_ADC_SetAnalogWDMonitChannels(ADC2, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC2, LL_ADC_AWD1, high_bc, low_bc);
    LL_ADC_SetAWDFilteringConfiguration(ADC2, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);

    // Clear all pending AWD flags first to prevent stale interrupt triggers
    LL_ADC_ClearFlag_AWD1(ADC1);
    LL_ADC_ClearFlag_AWD1(ADC2);

    // Now safely enable the AWD interrupts
    LL_ADC_EnableIT_AWD1(ADC1);
    LL_ADC_EnableIT_AWD1(ADC2);
}

void FOC_StateCalibration(void) {
    if (g_foc.adc_cal.cal_samples >= CAL_SAMPLES) {
        float n_ab = (float)(CAL_SAMPLES * 2 / 3);
        float n_c = (float)(CAL_SAMPLES / 3);

        float mean_a = (float)g_foc.adc_cal.offset_a / n_ab;
        float mean_b = (float)g_foc.adc_cal.offset_b / n_ab;
        float mean_c_pb1 = (float)g_foc.adc_cal.offset_c_pb1 / n_c;
        float mean_c_opamp3 = (float)g_foc.adc_cal.offset_c_opamp3 / n_c;

        g_foc.adc_cal.offset_a = (int32_t)(mean_a + 0.5f);
        g_foc.adc_cal.offset_b = (int32_t)(mean_b + 0.5f);
        g_foc.adc_cal.offset_c_pb1 = (int32_t)(mean_c_pb1 + 0.5f);
        g_foc.adc_cal.offset_c_opamp3 = (int32_t)(mean_c_opamp3 + 0.5f);

        /* Calculate Current Sensing RMS Noise & Peak-to-Peak Noise */
        float var_a = ((float)s_adc1_sq_accum / n_ab) - (mean_a * mean_a);
        float var_b = ((float)s_adc2_b_sq_accum / n_ab) - (mean_b * mean_b);
        float var_c_pb1 = ((float)s_adc1_c_pb1_sq_accum / n_c) - (mean_c_pb1 * mean_c_pb1);
        float var_c_opamp3 = ((float)s_adc2_c_opamp3_sq_accum / n_c) - (mean_c_opamp3 * mean_c_opamp3);
        if (var_a < 0.0f) var_a = 0.0f;
        if (var_b < 0.0f) var_b = 0.0f;
        if (var_c_pb1 < 0.0f) var_c_pb1 = 0.0f;
        if (var_c_opamp3 < 0.0f) var_c_opamp3 = 0.0f;

        float i_scale = ADC_Vref * fabsf(ADC_TO_CURRENT);
        float noise_rms_a = sqrtf(var_a) * i_scale;
        float noise_rms_b = sqrtf(var_b) * i_scale;
        float noise_rms_c = sqrtf(0.5f * (var_c_pb1 + var_c_opamp3)) * i_scale;
        float noise_rms = (noise_rms_a + noise_rms_b + noise_rms_c) * (1.0f / 3.0f);

        float pk_pk_a = (float)(s_adc1_max - s_adc1_min) * i_scale;
        float pk_pk_b = (float)(s_adc2_b_max - s_adc2_b_min) * i_scale;
        float pk_pk_c_pb1 = (float)(s_adc1_c_pb1_max - s_adc1_c_pb1_min) * i_scale;
        float pk_pk_c_opamp3 = (float)(s_adc2_c_opamp3_max - s_adc2_c_opamp3_min) * i_scale;
        float pk_pk_c = (pk_pk_c_pb1 > pk_pk_c_opamp3) ? pk_pk_c_pb1 : pk_pk_c_opamp3;
        float noise_pk_pk = pk_pk_a > pk_pk_b ? pk_pk_a : pk_pk_b;
        if (pk_pk_c > noise_pk_pk) noise_pk_pk = pk_pk_c;

        g_foc.noise_profile.noise_rms = noise_rms;
        g_foc.noise_profile.noise_pk_pk = noise_pk_pk;
        g_foc.noise_profile.is_flat_thr =
            clampf(0.5f * g_foc.noise_profile.noise_pk_pk, 0.003f, 0.500f);
        g_foc.noise_profile.i_inj_min = clampf(3.0f * noise_rms, 0.020f, 1.0f);
        g_foc.noise_profile.bemf_noise_sq = 1e-4f;

        /* Hardware Health Evaluation */
        if (noise_rms < 0.015f) {
            g_foc.noise_profile.health_status = 0; /* EXCELLENT (< 15mA) */
        } else if (noise_rms < 0.040f) {
            g_foc.noise_profile.health_status = 1; /* GOOD (15 - 40mA) */
        } else if (noise_rms < 0.100f) {
            g_foc.noise_profile.health_status = 2; /* NOISY (40 - 100mA) */
        } else {
            g_foc.noise_profile.health_status = 3; /* FAULT (> 100mA) */
        }

        /* Restore hardware to default skip C sensing after calibration */
        FOC_HW_SwitchCurrentSensing(2);

        /* Check if peak-to-peak variation on any phase is above threshold (30 ADC counts)
         * to detect if the motor is already spinning. */
        uint16_t diff_a = s_vphase_a_max - s_vphase_a_min;
        uint16_t diff_c = s_vphase_c_max - s_vphase_c_min;

        uint8_t is_spinning = (diff_a > 30) || (diff_c > 30);
        if (!is_spinning) {
            g_foc.adc_cal.offset_vphase_a = s_vphase_a_accum / CAL_SAMPLES;
            g_foc.adc_cal.offset_vphase_c = s_vphase_c_accum / CAL_SAMPLES;
        }

        // Configure hardware watchdogs dynamically based on offsets
        FOC_ConfigureAWD();

        g_foc.startup.theta = 0.0f;
        g_foc.startup.omega = 0.0f;
        g_foc.startup.counter = 0;

        /* Auto-calculate physical minimum observer speed based on freshly calibrated noise &
         * deadtime */
        g_foc.cfg.motor_min_spd = FOC_CalculateObserverMinSpeed();

        if (id_result.state != MOTOR_ID_STATE_ALIGN) {
            g_foc.status.state = FOC_STATE_DETECT;
        } else {
            FOC_EnableDrivers(1);

            /* Re-initialize MotorID now that noise_rms has been properly calibrated */
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
    float noise_in_band = g_foc.noise_profile.noise_rms * k_atten;
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
    } else {
        float flux_fallback = 60.0f / (1.732f * g_foc.cfg.motor_kv * TWO_PI * (float)g_foc.cfg.motor_poles);
        rec_handoff_omega = (flux_fallback > 1e-6f) ? (e_bemf_target / flux_fallback) : 700.0f;
    }
    /* 7. Safety constraints:
     * - Minimum 15 Hz electrical frequency for clean STF/PLL tracking (~94.25 rad/s elec)
     * - Maximum clamp to 25% of max rated speed */
    float min_elec_omega = 15.0f * TWO_PI;
    if (rec_handoff_omega < min_elec_omega) {
        rec_handoff_omega = min_elec_omega;
    }
    float max_handoff_limit = 0.25f * g_foc.cfg.motor_max_spd;
    if (rec_handoff_omega > max_handoff_limit) {
        rec_handoff_omega = max_handoff_limit;
    }
    return rec_handoff_omega;
}
