#ifndef SBV_OTA_H
#define SBV_OTA_H

#include "sbv.h"

#ifdef STM32F1xx
#define SBV_OTA_SLOT_NO         (2)
#define SBV_OTA_INVALID_SLOT    (0xFFu)

#define SBV_OTA_FW_SLOT_PAGES   (25)
#define SBV_OTA_GEN_CFG_PAGES   (1)

#define SBV_OTA_PAGES_SIZE      (1024)
#define SBV_OTA_SLOT_MAX_SIZE   (SBV_OTA_FW_SLOT_PAGES * SBV_OTA_PAGES_SIZE)

/*
 * The 64k FLASH memory layout of the SBV system
 *    Bootloader:     0x80000000 - 0x08003000 (12 KB)
 *    App slot 0:     0x08003000 - 0X08009400 (25 KB)
 *    App slot 1:     0X08009400 - 0x0800F800 (25 KB)
 *    Global config:  0x0800F800 - 0x80010000 (2 KB)
 */
#define SBV_OTA_CONFIG_FLASH_ADD    (0x0800F800)
#define SBV_OTA_SLOT0_FLASH_ADD     (0x08003000)
#define SBV_OTA_SLOT1_FLASH_ADD     (0X08009400)
#endif /*STM32F1xx*/

#define SBV_OTA_RCV_ACK       (1 << 0)
#define SBV_OTA_RCV_NACK      (1 << 1)
#define SBV_OTA_RCV_START     (1 << 2)
#define SBV_OTA_RCV_METADATA  (1 << 3)
#define SBV_OTA_RCV_PAGE      (1 << 4)
#define SBV_OTA_RCV_ALL       (1 << 5)
#define SBV_OTA_RCV_REPORT    (1 << 6)
#define SBV_OTA_RCV_ABORT     (1 << 7)

typedef struct sbv_ota_fw_metadata_t
{
  uint32_t  fw_size;
  uint32_t  fw_crc;
  char      fw_timestamp[20];
  char      fw_version[12];
} sbv_ota_fw_metadata_t;

/* Slot configuration */
typedef struct sbv_ota_slot_t
{
  uint8_t               is_slot_valid;
  uint8_t               is_slot_active;
  uint8_t               is_slot_update;
  sbv_ota_fw_metadata_t metadata;
} sbv_ota_slot_t;

typedef enum sbv_ot_reboot_reason_t
{
  SBV_OTA_POWER_UP_BOOT,
  SBV_OTA_NEW_UPDATE_BOOT
} sbv_ot_reboot_reason_t;

/* General configuration */
typedef struct
{
  sbv_ot_reboot_reason_t  reboot_reason;
  sbv_ota_slot_t          slot_table[SBV_OTA_SLOT_NO];
} sbv_ota_general_cfg;

typedef struct sbv_ota_tx_thread_param_t
{
  uint32_t  data_length;
  uint8_t   *data;
} sbv_ota_tx_thread_param_t;

#define SBV_OTA_DB_MUTEX_LOCK \
        sbv_rtos_mutex_lock(SBV_OTA_DB_MUTEX)

#define SBV_OTA_DB_MUTEX_UNLOCK \
        sbv_rtos_mutex_unlock(SBV_OTA_DB_MUTEX)

#define SBV_OTA_SLOT_PAGE_ADDR(SLOT) \
        (SLOT == 0) ? SBV_OTA_SLOT0_FLASH_ADD : SBV_OTA_SLOT1_FLASH_ADD

void
sbv_ota_update_init(void);
uint8_t
sbv_ota_get_available_slot_num (void);
int
sbv_ota_write_flash_data (uint8_t *data, uint32_t data_length, uint32_t page_addr);
void
sbv_ota_load_new_app (void);


#endif /*SBV_OTA_H*/