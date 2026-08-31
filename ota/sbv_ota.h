#ifndef SBV_OTA_H
#define SBV_OTA_H

typedef struct sbv_ota_installer_t
{
  sbv_rtos_queue_handle_t rx_queue;
  sbv_rtos_queue_handle_t tx_queue;
  sbv_rtos_mutex_t        mutex;
  sbv_ota_state_t         state;
  uint32_t                current_flash_page_addr;
  uint8_t                 data[sizeof(sbv_ota_system_msg_t)];
  uint8_t                 is_update_enable;
  uint8_t                 is_updating;
} sbv_ota_installer_t;

void
sbv_ota_update_init(void);

#endif /*SBV_OTA_H*/