/**
 * @file flash_config.c
 * @brief Persistent configuration storage using internal Flash
 *
 * Uses the last 2KB page of STM32G431CB Flash (page 63, 0x0801F800).
 * Data is written as 64-bit double-words (STM32G4 requirement).
 * Software CRC32 validates data integrity on load.
 */

#include "flash_config.h"

#include <string.h>

#include "foc_calibration.h"
#include "foc_config.h"
#include "foc_state_machine.h"
#include "main.h"
#include "pi_controller.h"
#include "smo_observer.h"
#include "stm32g4xx_ll_tim.h"

/*===========================================================================*/
/* Private                                                                   */
/*===========================================================================*/

/** Global config instance (lives in RAM) */
static FlashConfig_t g_config;

/** Pointer to config stored in Flash (read-only memory-mapped) */
#define FLASH_CONFIG_PTR ((const FlashConfig_t*)FLASH_CONFIG_ADDR)

/*===========================================================================*/
/* Software CRC32 (polynomial 0x04C11DB7)                                    */
/*===========================================================================*/

static uint32_t sw_crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

/**
 * @brief Compute CRC32 over config struct (excludes the crc field itself)
 */
static uint32_t config_compute_crc(const FlashConfig_t* cfg) {
    /* CRC covers everything except the last 4 bytes (the crc field) */
    return sw_crc32((const uint8_t*)cfg, sizeof(FlashConfig_t) - sizeof(uint32_t));
}

/**
 * @brief Validate a config struct
 * @return 1 if valid, 0 if invalid
 */
static int config_validate(const FlashConfig_t* cfg) {
    if (cfg->magic != FLASH_CONFIG_MAGIC) return 0;
    if (cfg->version != FLASH_CONFIG_VERSION) return 0;
    if (cfg->size != sizeof(FlashConfig_t)) return 0;
    if (cfg->crc != config_compute_crc(cfg)) return 0;
    return 1;
}

/*===========================================================================*/
/* Flash Operations                                                          */
/*===========================================================================*/

/**
 * @brief Erase the config Flash page
 * @return 0 on success, -1 on failure
 */
static int flash_erase_config_page(void) {
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = FLASH_CONFIG_PAGE;
    erase.NbPages = 1;

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief Write config struct to Flash (64-bit aligned writes)
 * @return 0 on success, -1 on failure
 */
static int flash_write_config(const FlashConfig_t* cfg) {
    const uint64_t* src = (const uint64_t*)cfg;
    uint32_t addr = FLASH_CONFIG_ADDR;
    uint32_t words = (sizeof(FlashConfig_t) + 7) / 8; /* Round up to 64-bit */

    for (uint32_t i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]) != HAL_OK) {
            return -1;
        }
        addr += 8;
    }
    return 0;
}

/*===========================================================================*/
/* Public API                                                                */
/*===========================================================================*/

void FlashConfig_LoadDefaults(void) {
    memset(&g_config, 0, sizeof(g_config));

    g_config.magic = FLASH_CONFIG_MAGIC;
    g_config.version = FLASH_CONFIG_VERSION;
    g_config.size = sizeof(FlashConfig_t);

    /* Auto-assign Default Values from X-Macro table */
#define PARAM_DEF(id, type, name, default_val) g_config.name = default_val;
#include "param_table.def"

    g_config.crc = config_compute_crc(&g_config);
}

void FlashConfig_Init(void) {
    if (config_validate(FLASH_CONFIG_PTR)) {
        memcpy(&g_config, FLASH_CONFIG_PTR, sizeof(FlashConfig_t));
    } else {
        FlashConfig_LoadDefaults();
        FlashConfig_Save();
    }
}

void FlashConfig_Apply(void) {
    /* Preserve valid runtime ADC calibration if g_config doesn't have it */
    if (g_foc.cfg.cal_valid && !g_config.cal_valid) {
        g_config.cal_offset_a = g_foc.cfg.cal_offset_a;
        g_config.cal_offset_b = g_foc.cfg.cal_offset_b;
        g_config.cal_offset_c_pb1 = g_foc.cfg.cal_offset_c_pb1;
        g_config.cal_offset_c_op3 = g_foc.cfg.cal_offset_c_op3;
        g_config.cal_vphase_a = g_foc.cfg.cal_vphase_a;
        g_config.cal_vphase_c = g_foc.cfg.cal_vphase_c;
        g_config.cal_noise_rms = g_foc.cfg.cal_noise_rms;
        g_config.cal_noise_pk_pk = g_foc.cfg.cal_noise_pk_pk;
        g_config.cal_is_flat_thr = g_foc.cfg.cal_is_flat_thr;
        g_config.cal_i_inj_min = g_foc.cfg.cal_i_inj_min;
        g_config.cal_valid = 1;
    }

    FlashConfig_t tmp_cfg;
    memcpy(&tmp_cfg, &g_config, sizeof(FlashConfig_t));

    if (tmp_cfg.pwm_frequency < 16000.0f || tmp_cfg.pwm_frequency > 96000.0f) {
        tmp_cfg.pwm_frequency = 48000.0f;
    }
    if (tmp_cfg.beep_volume > 100U) {
        tmp_cfg.beep_volume = 100U;
    }
    tmp_cfg.overmod_enable = (tmp_cfg.overmod_enable != 0U) ? 1U : 0U;

    float i_limit = tmp_cfg.motor_max_curr;
    float v_limit = g_foc.data.Vbus * SQRT3_INV;
    if (tmp_cfg.bw_cur < 1.0f) {
        float target_bw = tmp_cfg.pwm_frequency * (1.0f / 15.0f);
        float noise_rms = (tmp_cfg.cal_noise_rms > 0.01f) ? tmp_cfg.cal_noise_rms : 0.035f;
        float kp_limit = (0.025f * v_limit) / noise_rms;
        float bw_noise_limit = kp_limit / (TWO_PI * tmp_cfg.motor_ls);
        tmp_cfg.bw_cur = clampf(bw_noise_limit, 400.0f, target_bw);
    }

    if (tmp_cfg.kp_id < 1e-6f || tmp_cfg.ki_id < 1e-6f || tmp_cfg.kp_iq < 1e-6f ||
        tmp_cfg.ki_iq < 1e-6f) {
        tmp_cfg.kp_id = TWO_PI * tmp_cfg.bw_cur * tmp_cfg.motor_ls;
        tmp_cfg.ki_id = TWO_PI * tmp_cfg.bw_cur * tmp_cfg.motor_rs;
        tmp_cfg.kp_iq = tmp_cfg.kp_id;
        tmp_cfg.ki_iq = tmp_cfg.ki_id;
    }

    float bw_obs = 3000.0f;
    if (tmp_cfg.motor_ls > 1e-7f) {
        float f_elec_max =
            (tmp_cfg.motor_kv * g_foc.data.Vbus * tmp_cfg.motor_poles) * (1.0f / 60.0f);
        float f_motor = tmp_cfg.motor_rs / (TWO_PI * tmp_cfg.motor_ls);
        float bw_obs_target = fmaxf(2.0f * f_motor, 2.5f * f_elec_max);
        bw_obs_target = fmaxf(bw_obs_target, 1.5f * tmp_cfg.bw_cur);
        float max_obs_hw = tmp_cfg.pwm_frequency * (1.0f / 6.0f);
        bw_obs = clampf(bw_obs_target, 1.5f * f_motor, max_obs_hw);
    }

    uint32_t timer_arr = (uint32_t)((float)SYSCLK_FREQ / (2.0f * tmp_cfg.pwm_frequency) + 0.5f);
    float timer_arr_f = (float)timer_arr;
    float dt = 1.0f / tmp_cfg.pwm_frequency;
    float deadtime_duty = tmp_cfg.deadtime_ns * 1e-9f * tmp_cfg.pwm_frequency;
    float i_sq_lpf_alpha = clampf(480.0f * dt, 0.002f, 0.25f);
    float wc_current_stf;
    if (tmp_cfg.motor_ls > 1e-7f) {
        float f_bw = (tmp_cfg.kp_iq / tmp_cfg.motor_ls) * (1.0f / TWO_PI);
        wc_current_stf = TWO_PI * clampf(3.0f * f_bw, 500.0f, 4000.0f);
    } else {
        wc_current_stf = TWO_PI * 1500.0f;
    }

    /* Compute hardware-level constraints (ADC trigger shifting & 2-Zone Max Duty) */
    float margin = tmp_cfg.adc_margin_ticks + DEAD_TIME_NS * 1e-9f * (float)SYSCLK_FREQ;
    float total_sampling_ticks = margin + ADC_TICKS * 2.0f / 3.0f;
    float max_duty = clampf(1.0f - total_sampling_ticks / timer_arr_f, 0.50f, MAX_DUTY_HIGH);
    float v_max_scale_z1 = SQRT3_INV * max_duty;
    float v_max_scale_z2 = fminf((2.0f / 3.0f) * max_duty, SQRT3_INV);
    float v_max_scale = (tmp_cfg.overmod_enable != 0U) ? v_max_scale_z2 : v_max_scale_z1;
    uint32_t trigger = timer_arr - ADC_TICKS / 3;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    memcpy(&g_foc.cfg, &tmp_cfg, sizeof(FlashConfig_t));
    g_foc.timer_arr = timer_arr;
    g_foc.timer_arr_f = timer_arr_f;
    g_foc.dt = dt;
    g_foc.deadtime_duty = deadtime_duty;
    g_foc.wc_current_stf = wc_current_stf;
    g_foc.i_sq_lpf_alpha = i_sq_lpf_alpha;
    g_foc.max_duty = max_duty;
    g_foc.v_max_scale_z1 = v_max_scale_z1;
    g_foc.v_max_scale_z2 = v_max_scale_z2;
    g_foc.v_max_scale = v_max_scale;
    g_foc.v_max = g_foc.data.Vbus * v_max_scale;

    PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
    PI_SetLimits(&g_foc.ctrl.id, -g_foc.v_max, g_foc.v_max);
    PI_SetIntLimits(&g_foc.ctrl.id, -g_foc.v_max, g_foc.v_max);
    PI_SetTiming(&g_foc.ctrl.id, g_foc.dt);

    PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);
    PI_SetLimits(&g_foc.ctrl.iq, -g_foc.v_max, g_foc.v_max);
    PI_SetIntLimits(&g_foc.ctrl.iq, -g_foc.v_max, g_foc.v_max);
    PI_SetTiming(&g_foc.ctrl.iq, g_foc.dt);

    FOC_SyncDerivedParams();

    LADRC_SetGains(&g_foc.ctrl.speed, g_foc.cfg.ladrc_omega_c, g_foc.cfg.ladrc_omega_o,
                   g_foc.cfg.ladrc_b0);
    LADRC_SetLimits(&g_foc.ctrl.speed, -i_limit, i_limit);

    /* Update SMO Observer - Only re-init if motor is stopped */
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_FAULT) {
        SMO_Init(&g_foc.ctrl.smo);
    }
    SMO_SetTiming(&g_foc.ctrl.smo, g_foc.dt);

    SMO_SetMotorParams(&g_foc.ctrl.smo, g_foc.cfg.motor_rs, g_foc.cfg.motor_ls,
                       g_foc.cfg.motor_alpha, g_foc.cfg.motor_flux, g_foc.cfg.motor_poles,
                       g_foc.cfg.motor_max_spd, bw_obs);

    LL_TIM_OC_SetCompareCH4(TIM1, trigger);
    LL_TIM_SetAutoReload(TIM1, g_foc.timer_arr);

    if (!primask) {
        __enable_irq();
    }

    /* Update hardware watchdogs dynamically with the new safety limits */
    FOC_ConfigureAWD();
}

int FlashConfig_Save(void) {
    /* Auto-commit current runtime ADC calibration offsets & noise profile to Flash struct if valid
     */
    if (g_foc.cfg.cal_valid) {
        g_config.cal_offset_a = g_foc.cfg.cal_offset_a;
        g_config.cal_offset_b = g_foc.cfg.cal_offset_b;
        g_config.cal_offset_c_pb1 = g_foc.cfg.cal_offset_c_pb1;
        g_config.cal_offset_c_op3 = g_foc.cfg.cal_offset_c_op3;
        g_config.cal_vphase_a = g_foc.cfg.cal_vphase_a;
        g_config.cal_vphase_c = g_foc.cfg.cal_vphase_c;
        g_config.cal_noise_rms = g_foc.cfg.cal_noise_rms;
        g_config.cal_noise_pk_pk = g_foc.cfg.cal_noise_pk_pk;
        g_config.cal_is_flat_thr = g_foc.cfg.cal_is_flat_thr;
        g_config.cal_i_inj_min = g_foc.cfg.cal_i_inj_min;
        g_config.cal_valid = 1;
    }

    g_config.crc = config_compute_crc(&g_config);

    if (FLASH_CONFIG_PTR->magic == FLASH_CONFIG_MAGIC && FLASH_CONFIG_PTR->crc == g_config.crc &&
        memcmp(FLASH_CONFIG_PTR, &g_config, sizeof(FlashConfig_t)) == 0) {
        return 0;
    }

    LL_IWDG_ReloadCounter(IWDG);

    __disable_irq();

    HAL_FLASH_Unlock();

    int result = 0;
    if (flash_erase_config_page() != 0) {
        result = -1;
    } else if (flash_write_config(&g_config) != 0) {
        result = -1;
    }

    HAL_FLASH_Lock();
    __enable_irq();

    return result;
}

FlashConfig_t* FlashConfig_Get(void) {
    return &g_config;
}
