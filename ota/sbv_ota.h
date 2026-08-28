#ifndef SBV_OTA_H
#define SBV_OTA_H

typedef struct sbv_ota_tx_thread_param_t
{
  uint32_t  data_length;
  uint8_t   *data;
} sbv_ota_tx_thread_param_t;

#define SBV_OTA_DB_MUTEX_LOCK \
        sbv_rtos_mutex_lock(SBV_OTA_DB_MUTEX)

#define SBV_OTA_DB_MUTEX_UNLOCK \
        sbv_rtos_mutex_unlock(SBV_OTA_DB_MUTEX)

void
sbv_ota_update_init(void);
uint8_t
sbv_ota_get_available_slot_num (void);

#endif /*SBV_OTA_H*/