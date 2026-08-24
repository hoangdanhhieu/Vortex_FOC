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

#include "foc_config.h"
#include "foc_state_machine.h"
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
    memcpy(&g_foc.cfg, &g_config, sizeof(FlashConfig_t));

    FOC_SetDirection(g_foc.cfg.direction);

    float i_limit = g_foc.cfg.motor_max_curr;
    float v_limit = g_foc.data.Vbus * SQRT3_INV; /* Vbus / sqrt(3) */

    PI_SetGains(&g_foc.ctrl.id, g_foc.cfg.kp_id, g_foc.cfg.ki_id);
    PI_SetLimits(&g_foc.ctrl.id, -v_limit, v_limit);
    PI_SetIntLimits(&g_foc.ctrl.id, -v_limit, v_limit);

    PI_SetGains(&g_foc.ctrl.iq, g_foc.cfg.kp_iq, g_foc.cfg.ki_iq);
    PI_SetLimits(&g_foc.ctrl.iq, -v_limit, v_limit);
    PI_SetIntLimits(&g_foc.ctrl.iq, -v_limit, v_limit);

    LADRC_SetGains(&g_foc.ctrl.speed, g_foc.cfg.ladrc_omega_c, g_foc.cfg.ladrc_omega_o,
                   g_foc.cfg.ladrc_b0);
    LADRC_SetLimits(&g_foc.ctrl.speed, -i_limit, i_limit);

    /* Update SMO Observer - Only re-init if motor is stopped */
    if (g_foc.status.state == FOC_STATE_IDLE || g_foc.status.state == FOC_STATE_STOP ||
        g_foc.status.state == FOC_STATE_FAULT) {
        SMO_Init(&g_foc.ctrl.smo);
    }

    SMO_SetMotorParams(&g_foc.ctrl.smo, g_foc.cfg.motor_rs, g_foc.cfg.motor_ls,
                       g_foc.cfg.motor_alpha, g_foc.cfg.motor_flux, g_foc.cfg.motor_poles,
                       g_foc.cfg.motor_max_spd, g_foc.cfg.motor_min_spd);

    /* Update hardware-level constraints (ADC trigger shifting & Max Duty) */
    float margin = g_foc.cfg.adc_margin_ticks + DEAD_TIME_NS * 1e-9f * (float)SYSCLK_FREQ;
    float total_sampling_ticks = margin + ADC_TICKS / 2.0f;
    float max_duty = 1.0f - total_sampling_ticks / (float)TIM1_ARR;

    /* Clamp max duty to safe operating region [10%, 99%] */
    g_foc.max_duty = clampf(max_duty, 0.10f, 0.99f);

    uint32_t trigger = TIM1_ARR - ADC_TICKS / 2;
    LL_TIM_OC_SetCompareCH4(TIM1, trigger);

    /* Update hardware watchdogs dynamically with the new safety limits */
    FOC_ConfigureAWD();
}

int FlashConfig_Save(void) {
    /* Update CRC before saving */
    g_config.crc = config_compute_crc(&g_config);

    /* Disable interrupts during Flash operations */
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
