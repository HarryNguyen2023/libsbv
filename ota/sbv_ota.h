#ifndef SBV_OTA_H
#define SBV_OTA_H

typedef struct sbv_ota_installer_t
{
  sbv_rtos_queue_handle_t rx_queue;
  sbv_rtos_queue_handle_t tx_queue;
  sbv_rtos_mutex_t        mutex;

  uint32_t                current_flash_page_addr;
  uint8_t                 data[sizeof(sbv_ota_system_msg_t)];

  sbv_ota_fw_metadata_t   fw_metadata;
  uint8_t                 *fw_data_addr;

  uint8_t                 inactive_slot;
  uint32_t                slot_pag_add;

  uint8_t                 is_update_enable;
  uint8_t                 is_updating;
} sbv_ota_installer_t;

void
sbv_ota_update_init(void *param);
int
sbv_ota_ipc_queue_init (sbv_ota_ipc_t *ipc);

#endif /*SBV_OTA_H*/