#ifndef SBV_OTA_COMMON_H
#define SBV_OTA_COMMON_H

#ifdef STM32F1xx
#define SBV_OTA_SLOT_NO           2
#define SBV_OTA_INVALID_SLOT      (0xFFu)

#define SBV_OTA_FW_SLOT_PAGES     25
#define SBV_OTA_GEN_CFG_PAGES     2
#define SBV_OTA_BOOTLOADER_PAGES  12

#define SBV_OTA_PAGES_SIZE        1024
#define SBV_OTA_SLOT_MAX_SIZE     (SBV_OTA_FW_SLOT_PAGES * SBV_OTA_PAGES_SIZE)

/*
 * The 64k FLASH memory layout of the SBV system
 *    Bootloader:     0x80000000 - 0x08003000 (12 KB)
 *    App slot 0:     0x08003000 - 0X08009400 (25 KB)
 *    App slot 1:     0X08009400 - 0x0800F800 (25 KB)
 *    Global config:  0x0800F800 - 0x80010000 (2 KB)
 */
#define SBV_OTA_CONFIG_FLASH_ADD    (0x0800F800u)
#define SBV_OTA_SLOT0_FLASH_ADD     (0x08003000u)
#define SBV_OTA_SLOT1_FLASH_ADD     (0X08009400u)
#else
#define SBV_OTA_SLOT_NO             (0)
#endif /*STM32F1xx*/

#define SBV_OTA_DATA_MAX_SIZE       (1024)

#define SBV_OTA_SLOT_PAGE_ADDR(SLOT) \
        (SLOT == 0) ? SBV_OTA_SLOT0_FLASH_ADD : SBV_OTA_SLOT1_FLASH_ADD

#define SBV_OTA_FW_VERSION_LENGTH   12
#define SBV_OTA_FW_TIME_LENGTH      20

#define SVB_OTA_SEQ_DUP             (-3)

#define SBV_OTA_UID_BASE            0x1FFFF7E8UL

typedef struct sbv_ota_fw_version_t
{
  uint8_t  major;   // A
  uint8_t  minor;   // B
  uint16_t build;   // CDEF
} sbv_ota_fw_version_t;

typedef struct sbv_ota_fw_metadata_t
{
  uint32_t  fw_size;
  uint32_t  fw_crc;
  char      fw_timestamp[SBV_OTA_FW_TIME_LENGTH];
  char      fw_version[SBV_OTA_FW_VERSION_LENGTH];
} sbv_ota_fw_metadata_t;

/* Slot configuration */
typedef struct sbv_ota_slot_t
{
  uint8_t               is_slot_valid;
  uint8_t               is_slot_active;
  uint8_t               is_slot_update;
  sbv_ota_fw_metadata_t metadata;
} sbv_ota_slot_t;

typedef enum sbv_ota_reboot_reason_t
{
  SBV_OTA_POWER_UP_BOOT,
  SBV_OTA_NEW_UPDATE_BOOT
} sbv_ota_reboot_reason_t;

/* General configuration */
typedef struct sbv_ota_general_cfg_t
{
  uint32_t                 magic;
  sbv_ota_reboot_reason_t  reboot_reason;
  sbv_ota_slot_t           slot_table[SBV_OTA_SLOT_NO];
  uint32_t                 crc;     /* CRC of everything above */
} __attribute__((packed)) sbv_ota_general_cfg_t;

/* State of the OTA process FSM */
typedef enum sbv_ota_state_t
{
	SBV_OTA_STATE_IDLE,
	SBV_OTA_STATE_START,
	SBV_OTA_STATE_HEADER,
	SBV_OTA_STATE_DATA,
	SBV_OTA_STATE_END,
  SBV_OTA_STATE_MAX,
} sbv_ota_state_t;

typedef enum sbv_ota_system_msg_event_t
{
  SBV_OTA_EVENT_UDP_START,
  SBV_OTA_EVENT_IMG_WRITE,
  SBV_OTA_EVENT_UDP_FINALIZE,
  SBV_OTA_EVENT_ACK,
  SBV_OTA_EVENT_ABORT
} sbv_ota_system_msg_event_t;

typedef struct sbv_ota_system_msg_t
{
  sbv_ota_system_msg_event_t event;
  void*                      data;
} sbv_ota_system_msg_t;

typedef struct sbv_ota_ipc_t {
  sbv_rtos_queue_handle_t to_installer;
  sbv_rtos_queue_handle_t to_slave_fsm;
} sbv_ota_ipc_t;

int
sbv_ota_erase_flash_data (uint32_t page_addr, uint16_t pages_num);
int
sbv_ota_write_flash_data (uint8_t *data, uint32_t data_length, uint32_t page_addr);
int
sbv_ota_cfg_read_and_validate (sbv_ota_general_cfg_t *c);
int
sbv_ota_cfg_commit(sbv_ota_general_cfg_t *c);
uint8_t
sbv_ota_get_update_slot (sbv_ota_general_cfg_t* cfg);
uint8_t
sbv_ota_get_active_slot (sbv_ota_general_cfg_t* cfg);
uint8_t
sbv_ota_get_available_slot_num (void);
int
sbv_ota_get_current_fw_metadata (sbv_ota_fw_metadata_t* current_fw_medata);
uint32_t
sbv_ota_calculate_crc (const uint8_t *buffer, const uint32_t buffer_length);
uint32_t
sbv_ota_frame_crc (uint8_t *buffer, uint32_t buffer_length);
int
sbv_ota_fw_image_crc_validate (const uint32_t page_addr, const sbv_ota_fw_metadata_t fw_metadata);
int
sbv_ota_is_valid_page_addr(const uint32_t slot_pag_addr);
int
sbv_ota_is_valid_fw_slot(uint16_t image_slot);

int
sbv_ota_send_system_msg (sbv_rtos_queue_handle_t queue, sbv_ota_system_msg_event_t event,
                         void *data, uint16_t timeout_ms);
uint16_t
sbv_ota_get_random_seq_number (void);
int
sbv_ota_seq_num_validate (uint16_t* curr_seq_num, uint16_t new_seq_num, uint16_t seq_num_offset);
void
sbv_ota_random_init_unique(void);
uint8_t
sbv_ota_fw_version_encode(char *str, sbv_ota_fw_version_t *ver);
uint8_t
sbv_ota_fw_version_decode(const char *str, sbv_ota_fw_version_t *ver);
int
sbv_ota_fw_version_compare(const sbv_ota_fw_version_t *v1, const sbv_ota_fw_version_t *v2);
#endif /* SBV_OTA_COMMON_H */