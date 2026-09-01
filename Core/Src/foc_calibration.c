/**
 * @file foc_calibration.c
 * @brief FOC ADC Offset Calibration, Noise Characterization & AWD Protection Implementation
 */

#include "foc_calibration.h"

#include "foc_config.h"
#include "foc_state_machine.h"
#include "math.h"
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
static uint64_t s_adc2_c_sq_accum = 0;
static uint16_t s_adc1_min = 4096;
static uint16_t s_adc1_max = 0;
static uint16_t s_adc2_b_min = 4096;
static uint16_t s_adc2_b_max = 0;
static uint16_t s_adc2_c_min = 4096;
static uint16_t s_adc2_c_max = 0;

extern volatile uint16_t adc_regular_buffer[2];
extern volatile float ADC_Vref;

void FOC_Calibration_Reset(void) {
    g_foc.adc_cal.offset_a = 0;
    g_foc.adc_cal.offset_b = 0;
    g_foc.adc_cal.offset_c = 0;
    g_foc.adc_cal.cal_samples = 0;

    s_vphase_a_accum = 0;
    s_vphase_c_accum = 0;
    s_vphase_a_min = 4096;
    s_vphase_a_max = 0;
    s_vphase_c_min = 4096;
    s_vphase_c_max = 0;

    s_adc1_sq_accum = 0;
    s_adc2_b_sq_accum = 0;
    s_adc2_c_sq_accum = 0;
    s_adc1_min = 4096;
    s_adc1_max = 0;
    s_adc2_b_min = 4096;
    s_adc2_b_max = 0;
    s_adc2_c_min = 4096;
    s_adc2_c_max = 0;
}

void FOC_Calibration_Accumulate(uint16_t adc1_data, uint16_t adc2_data) {
    /* ADC1 always measures Phase A */
    g_foc.adc_cal.offset_a += adc1_data;
    s_adc1_sq_accum += (uint64_t)adc1_data * adc1_data;
    if (adc1_data < s_adc1_min) s_adc1_min = adc1_data;
    if (adc1_data > s_adc1_max) s_adc1_max = adc1_data;

    /* ADC2 alternates: even=Phase B (VOPAMP2), odd=Phase C (VOPAMP3) */
    if ((g_foc.adc_cal.cal_samples & 1) == 0) {
        g_foc.adc_cal.offset_b += adc2_data;
        s_adc2_b_sq_accum += (uint64_t)adc2_data * adc2_data;
        if (adc2_data < s_adc2_b_min) s_adc2_b_min = adc2_data;
        if (adc2_data > s_adc2_b_max) s_adc2_b_max = adc2_data;
    } else {
        g_foc.adc_cal.offset_c += adc2_data;
        s_adc2_c_sq_accum += (uint64_t)adc2_data * adc2_data;
        if (adc2_data < s_adc2_c_min) s_adc2_c_min = adc2_data;
        if (adc2_data > s_adc2_c_max) s_adc2_c_max = adc2_data;
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
        g_foc.adc_cal.offset_c < 200) {
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
    int32_t avg_offset_ac = (g_foc.adc_cal.offset_a + g_foc.adc_cal.offset_c) / 2;
    int32_t high_ac = avg_offset_ac + (int32_t)adc_step;
    int32_t low_ac = avg_offset_ac - (int32_t)adc_step;
    if (high_ac > 4095) high_ac = 4095;
    if (low_ac < 0) low_ac = 0;

    // Phase B and C (ADC2 AWD1 - monitors all injected channels: VOPAMP2 and VOPAMP3)
    int32_t avg_offset_bc = (g_foc.adc_cal.offset_b + g_foc.adc_cal.offset_c) / 2;
    int32_t high_bc = avg_offset_bc + (int32_t)adc_step;
    int32_t low_bc = avg_offset_bc - (int32_t)adc_step;
    if (high_bc > 4095) high_bc = 4095;
    if (low_bc < 0) low_bc = 0;

    // 2. Vbus protection is now handled in software (FOC_SlowTask at 1kHz)

    // 3. Program hardware registers using LL driver
    // ADC1: AWD1 for all Injected Channels (Phase A and Phase C)
    LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
    LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, high_ac, low_ac);
    LL_ADC_SetAWDFilteringConfiguration(ADC1, LL_ADC_AWD1, LL_ADC_AWD_FILTERING_3SAMPLES);

    // ADC2: AWD1 for all Injected Channels (Phase B and Phase C)
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
        float n_a = (float)CAL_SAMPLES;
        float n_bc = (float)(CAL_SAMPLES / 2.0);

        float mean_a = (float)g_foc.adc_cal.offset_a / n_a;
        float mean_b = (float)g_foc.adc_cal.offset_b / n_bc;
        float mean_c = (float)g_foc.adc_cal.offset_c / n_bc;

        g_foc.adc_cal.offset_a = (int32_t)(mean_a + 0.5f);
        g_foc.adc_cal.offset_b = (int32_t)(mean_b + 0.5f);
        g_foc.adc_cal.offset_c = (int32_t)(mean_c + 0.5f);

        /* Calculate Current Sensing RMS Noise & Peak-to-Peak Noise */
        float var_a = ((float)s_adc1_sq_accum / n_a) - (mean_a * mean_a);
        float var_b = ((float)s_adc2_b_sq_accum / n_bc) - (mean_b * mean_b);
        float var_c = ((float)s_adc2_c_sq_accum / n_bc) - (mean_c * mean_c);
        if (var_a < 0.0f) var_a = 0.0f;
        if (var_b < 0.0f) var_b = 0.0f;
        if (var_c < 0.0f) var_c = 0.0f;

        float i_scale = ADC_Vref * fabsf(ADC_TO_CURRENT);
        float noise_rms_a = sqrtf(var_a) * i_scale;
        float noise_rms_b = sqrtf(var_b) * i_scale;
        float noise_rms_c = sqrtf(var_c) * i_scale;
        float noise_rms = (noise_rms_a + noise_rms_b + noise_rms_c) * (1.0f / 3.0f);

        float pk_pk_a = (float)(s_adc1_max - s_adc1_min) * i_scale;
        float pk_pk_b = (float)(s_adc2_b_max - s_adc2_b_min) * i_scale;
        float pk_pk_c = (float)(s_adc2_c_max - s_adc2_c_min) * i_scale;
        float noise_pk_pk = pk_pk_a > pk_pk_b ? pk_pk_a : pk_pk_b;
        if (pk_pk_c > noise_pk_pk) noise_pk_pk = pk_pk_c;

        g_foc.noise_profile.noise_rms = noise_rms;
        g_foc.noise_profile.noise_pk_pk = 6.0f * noise_rms;
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

        /* Restore ADC2 to default (VOPAMP2) after calibration */
        LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);

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

        if (id_result.state != MOTOR_ID_STATE_ALIGN) {
            g_foc.status.state = FOC_STATE_DETECT;
        } else {
            FOC_SetPhaseVoltageDMA(0);
            FOC_EnableDrivers(1);
            
            /* Re-initialize MotorID now that noise_rms has been properly calibrated */
            MotorID_Start();
            
            g_foc.status.state = FOC_STATE_SELF_COMMISSION;
        }
    }
}
