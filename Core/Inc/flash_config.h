/**
 * @file flash_config.h
 * @brief Persistent configuration storage in Flash for FOC system
 *
 * Uses the last 2KB Flash page (0x0801F800) on STM32G431CB.
 * Table-driven design: add new parameters by extending FlashConfig_t
 * and the parameter table in uart_cmd.c.
 */

#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#include <stdint.h>

/*===========================================================================*/
/* Flash Layout Constants                                                    */
/*===========================================================================*/

/** Flash page used for config storage (last page of 128KB Flash) */
#define FLASH_CONFIG_PAGE 63
#define FLASH_CONFIG_ADDR 0x0801F800UL
#define FLASH_CONFIG_MAGIC 0xC0F1A500UL
#define FLASH_CONFIG_VERSION 8

/*===========================================================================*/
/* Configuration Structure                                                   */
/*===========================================================================*/

typedef struct {
    /* Header (validation) */
    uint32_t magic;   /**< Must be FLASH_CONFIG_MAGIC */
    uint16_t version; /**< Struct version for migration */
    uint16_t size;    /**< sizeof(FlashConfig_t) for validation */

    /* Auto-generate struct fields from X-Macro table */
#define PARAM_DEF(id, type, name, default_val) type name;
#include "param_table.def"

    /* Reserved for future parameters */
    uint8_t _reserved[4];

    /* Footer (integrity) */
    uint32_t crc; /**< CRC32 checksum of all preceding bytes */
} FlashConfig_t;

/*===========================================================================*/
/* Public API                                                                */
/*===========================================================================*/

/**
 * @brief Initialize config: load from Flash or use defaults if invalid
 */
void FlashConfig_Init(void);

/**
 * @brief Save current config to Flash
 * @return 0 on success, -1 on failure
 */
int FlashConfig_Save(void);

/**
 * @brief Reset config to compile-time defaults (does NOT save to Flash)
 */
void FlashConfig_LoadDefaults(void);

/**
 * @brief Apply config values to live FOC controllers (PI, SMO, faults, etc.)
 */
void FlashConfig_Apply(void);

/**
 * @brief Get pointer to the current config
 * @return Pointer to global config instance
 */
FlashConfig_t* FlashConfig_Get(void);

#endif /* FLASH_CONFIG_H */
