/**
 * @file foc_beep.c
 * @brief Table-driven, multi-frequency ESC beep sequencer & synthesizer implementation
 */

#include "foc_beep.h"

#include <math.h>
#include <stddef.h>

#include "cordic_math.h"

typedef struct {
    uint16_t freq_hz;     /**< 0 = silence/gap; >0 = tone frequency [Hz] */
    uint16_t duration_ms; /**< Note duration [ms] */
    uint8_t amp_pct;      /**< Relative amplitude scale [0..150%], 100 = nominal */
} FOC_BeepNote_t;

typedef struct {
    const FOC_BeepNote_t* notes;
    uint8_t len;
} FOC_BeepSeq_t;

static const FOC_BeepNote_t k_beep_power_on[] = {
    {1480U, 80U, 95U},  {0U, 35U, 0U},  {1976U, 80U, 100U},  {0U, 35U, 0U},
    {2637U, 85U, 105U}, {0U, 115U, 0U}, {3520U, 250U, 110U},
};

static const FOC_BeepNote_t k_beep_signal_lock[] = {
    {1319U, 75U, 95U},
    {0U, 35U, 0U},
    {1976U, 105U, 105U},
};

static const FOC_BeepNote_t k_beep_armed[] = {
    {2637U, 65U, 100U},
    {0U, 45U, 0U},
    {2637U, 100U, 110U},
};

static const FOC_BeepNote_t k_beep_beacon_1[] = {
    {2093U, 90U, 100U},
    {0U, 30U, 0U},
    {2637U, 130U, 110U},
};

static const FOC_BeepNote_t k_beep_beacon_2[] = {
    {3136U, 90U, 110U},
    {0U, 30U, 0U},
    {2093U, 140U, 100U},
};

static const FOC_BeepNote_t k_beep_beacon_3[] = {
    {3136U, 90U, 130U}, {2349U, 90U, 130U},  {3136U, 90U, 130U},
    {2349U, 90U, 130U}, {1760U, 160U, 130U},
};

static const FOC_BeepNote_t k_beep_beacon_4[] = {
    {2637U, 60U, 105U}, {0U, 35U, 0U}, {2637U, 60U, 105U}, {0U, 35U, 0U}, {3136U, 90U, 115U},
};

static const FOC_BeepNote_t k_beep_beacon_5[] = {
    {1319U, 120U, 110U},
    {0U, 30U, 0U},
    {1760U, 180U, 120U},
};

static const FOC_BeepNote_t k_beep_success[] = {
    {2093U, 65U, 95U}, {0U, 20U, 0U}, {2637U, 65U, 100U}, {0U, 20U, 0U}, {3136U, 110U, 105U},
};

static const FOC_BeepNote_t k_beep_error[] = {
    {784U, 140U, 100U},
    {0U, 60U, 0U},
    {523U, 160U, 100U},
};

static const FOC_BeepSeq_t k_beep_sequences[FOC_BEEP_COUNT] = {
    [FOC_BEEP_POWER_ON] = {k_beep_power_on,
                           (uint8_t)(sizeof(k_beep_power_on) / sizeof(k_beep_power_on[0]))},
    [FOC_BEEP_SIGNAL_LOCK] = {k_beep_signal_lock, (uint8_t)(sizeof(k_beep_signal_lock) /
                                                            sizeof(k_beep_signal_lock[0]))},
    [FOC_BEEP_ARMED] = {k_beep_armed, (uint8_t)(sizeof(k_beep_armed) / sizeof(k_beep_armed[0]))},
    [FOC_BEEP_BEACON_1] = {k_beep_beacon_1,
                           (uint8_t)(sizeof(k_beep_beacon_1) / sizeof(k_beep_beacon_1[0]))},
    [FOC_BEEP_BEACON_2] = {k_beep_beacon_2,
                           (uint8_t)(sizeof(k_beep_beacon_2) / sizeof(k_beep_beacon_2[0]))},
    [FOC_BEEP_BEACON_3] = {k_beep_beacon_3,
                           (uint8_t)(sizeof(k_beep_beacon_3) / sizeof(k_beep_beacon_3[0]))},
    [FOC_BEEP_BEACON_4] = {k_beep_beacon_4,
                           (uint8_t)(sizeof(k_beep_beacon_4) / sizeof(k_beep_beacon_4[0]))},
    [FOC_BEEP_BEACON_5] = {k_beep_beacon_5,
                           (uint8_t)(sizeof(k_beep_beacon_5) / sizeof(k_beep_beacon_5[0]))},
    [FOC_BEEP_SUCCESS] = {k_beep_success,
                          (uint8_t)(sizeof(k_beep_success) / sizeof(k_beep_success[0]))},
    [FOC_BEEP_ERROR] = {k_beep_error, (uint8_t)(sizeof(k_beep_error) / sizeof(k_beep_error[0]))},
};

static const float k_beep_phase_angles[3] = {0.0f, 0.6666667f, -0.6666667f};

typedef struct {
    const FOC_BeepNote_t* seq;
    uint8_t len;
    uint8_t seg_idx;
    uint32_t seg_tick;
    uint32_t seg_ticks;
    uint32_t fade_ticks;
    uint32_t total_tick;
    uint32_t max_ticks;
    float dt;
    float motor_rs;
    float motor_ls;
    float vol_scale;
    float v_dt_ff;
    float vbus;
    float phase;
    float phase_step;
    float v_amp;
    float theta;
    float inv_fade;
} FOC_BeepCtx_t;

static FOC_BeepCtx_t s_beep = {0};

static void FOC_BeepLoadNote(uint8_t idx) {
    const FOC_BeepNote_t* note = &s_beep.seq[idx];
    float dt = s_beep.dt;
    float f_isr = 1.0f / dt;

    uint32_t ticks = (uint32_t)((float)note->duration_ms * 0.001f * f_isr);
    if (ticks == 0u) {
        ticks = 1u;
    }
    s_beep.seg_ticks = ticks;

    uint32_t fade_ticks = (uint32_t)(BEEP_FADE_MS * 0.001f * f_isr);
    if (fade_ticks > (ticks >> 1)) {
        fade_ticks = ticks >> 1;
    }
    s_beep.fade_ticks = fade_ticks;
    s_beep.inv_fade = (fade_ticks > 0u) ? (1.0f / (float)fade_ticks) : 0.0f;

    s_beep.phase = 0.0f;
    s_beep.theta = k_beep_phase_angles[idx % 3u];

    if (note->freq_hz == 0u || note->amp_pct == 0u) {
        s_beep.phase_step = 0.0f;
        s_beep.v_amp = 0.0f;
        return;
    }

    float f_tone = (float)note->freq_hz;
    /* Guard against Nyquist aliasing / DC injection at lower PWM frequencies */
    while ((f_tone * 2.0f * dt) >= 0.85f && f_tone > 200.0f) {
        f_tone *= 0.5f;
    }
    s_beep.phase_step = f_tone * 2.0f * dt;

    float v_amp;
    if (s_beep.motor_rs > 1e-4f && s_beep.motor_ls > 1e-7f) {
        float i_target = BEEP_TARGET_CURR_MAX * s_beep.vol_scale;
        float omega_tone = TWO_PI * f_tone;
        float x_l = omega_tone * s_beep.motor_ls;
        float z_tone = sqrtf(s_beep.motor_rs * s_beep.motor_rs + x_l * x_l);
        v_amp = i_target * z_tone;
    } else {
        v_amp = BEEP_V_AMP_FALLBACK * s_beep.vol_scale;
    }

    v_amp = v_amp * ((float)note->amp_pct * 0.01f) + s_beep.v_dt_ff;
    float v_cap = fminf(BEEP_V_MAX_ABS, 0.25f * s_beep.vbus);
    s_beep.v_amp = clampf(v_amp, 0.0f, v_cap);
}

uint8_t FOC_Beep_Start(FOC_BeepType_t type, float dt, float motor_rs, float motor_ls, float vbus,
                       float deadtime_duty, uint8_t volume_pct) {
#if BEEP_ENABLE
    if (volume_pct == 0u || (uint8_t)type >= (uint8_t)FOC_BEEP_COUNT) {
        return 0u;
    }
    const FOC_BeepSeq_t* seq_desc = &k_beep_sequences[type];
    if (seq_desc->notes == NULL || seq_desc->len == 0u) {
        return 0u;
    }

    uint8_t clamped_vol = (volume_pct > 100u) ? 100u : volume_pct;
    float vol_norm = (float)clamped_vol * 0.01f;
    /* Perceptual ^1.5 curve preserves audible low-volume steps above MOSFET deadtime */
    float vol_scale = vol_norm * sqrtf(vol_norm);
    float dt_safe = (deadtime_duty > 0.0f) ? deadtime_duty : 0.0f;
    float vbus_safe = (vbus > 0.0f) ? vbus : 0.0f;
    /* Precompute d-axis deadtime voltage feedforward (tapered below 20% volume) */
    float dt_ramp = clampf(vol_norm * 5.0f, 0.0f, 1.0f);

    float safe_dt = (dt > 1e-7f) ? dt : CONTROL_PERIOD;
    uint32_t total_ms = 200u;
    for (uint8_t i = 0u; i < seq_desc->len; i++) {
        total_ms += seq_desc->notes[i].duration_ms;
    }

    s_beep.seq = seq_desc->notes;
    s_beep.len = seq_desc->len;
    s_beep.seg_idx = 0u;
    s_beep.seg_tick = 0u;
    s_beep.total_tick = 0u;
    s_beep.max_ticks = (uint32_t)((float)total_ms * 0.001f / safe_dt);
    s_beep.dt = safe_dt;
    s_beep.motor_rs = motor_rs;
    s_beep.motor_ls = motor_ls;
    s_beep.vol_scale = vol_scale;
    s_beep.v_dt_ff = dt_safe * vbus_safe * dt_ramp;
    s_beep.vbus = vbus;

    FOC_BeepLoadNote(0u);
    return 1u;
#else
    (void)type;
    (void)dt;
    (void)motor_rs;
    (void)motor_ls;
    (void)vbus;
    (void)deadtime_duty;
    (void)volume_pct;
    return 0u;
#endif
}

void FOC_Beep_Stop(void) {
    s_beep.seq = NULL;
    s_beep.len = 0u;
    s_beep.seg_idx = 0u;
    s_beep.seg_tick = 0u;
    s_beep.total_tick = 0u;
    s_beep.phase = 0.0f;
    s_beep.phase_step = 0.0f;
    s_beep.v_amp = 0.0f;
}

CCMRAM_FUNC FOC_BeepStepStatus_t FOC_Beep_Step(float* out_vd, float* out_theta) {
    if (s_beep.seq == NULL || s_beep.len == 0u || s_beep.total_tick >= s_beep.max_ticks) {
        FOC_Beep_Stop();
        return FOC_BEEP_STEP_DONE;
    }

    s_beep.total_tick++;
    s_beep.seg_tick++;

    if (s_beep.seg_tick > s_beep.seg_ticks) {
        uint8_t next_seg = (uint8_t)(s_beep.seg_idx + 1u);
        if (next_seg >= s_beep.len) {
            FOC_Beep_Stop();
            return FOC_BEEP_STEP_DONE;
        }
        s_beep.seg_idx = next_seg;
        s_beep.seg_tick = 1u;
        FOC_BeepLoadNote(next_seg);
    }

    if (s_beep.phase_step <= 0.0f) {
        return FOC_BEEP_STEP_SILENCE;
    }

    s_beep.phase += s_beep.phase_step;
    if (s_beep.phase >= 1.0f) {
        s_beep.phase -= 2.0f;
    }

    float cos_out, sin_out;
    cordic_sincos(s_beep.phase, &cos_out, &sin_out);

    /* Soft-clipped sine adds crisp harmonics and faster zero-crossings for deadtime compensation */
    float wave = clampf(2.5f * sin_out, -1.0f, 1.0f);

    float env = 1.0f;
    uint32_t ticks_left = s_beep.seg_ticks - s_beep.seg_tick;
    if (s_beep.fade_ticks > 0u && ticks_left < s_beep.fade_ticks) {
        env = (float)ticks_left * s_beep.inv_fade;
    }

    if (out_vd != NULL) {
        *out_vd = s_beep.v_amp * env * wave;
    }
    if (out_theta != NULL) {
        *out_theta = s_beep.theta;
    }
    return FOC_BEEP_STEP_TONE;
}
