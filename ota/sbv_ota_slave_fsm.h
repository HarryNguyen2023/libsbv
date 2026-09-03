#ifndef SBV_OTA_SLAVE_FSM_T
#define SBV_OTA_SLAVE_FSM_T

typedef struct sbv_ota_msg_slave_handler_t
{
  sbv_cqbuff*               data_queue;
  sbv_rtos_mutex_t          mu;

  sbv_ota_state_t           state;
  sbv_ota_state_t           next_state;

  sbv_ota_fw_metadata_t     new_fw_metadata;
  uint8_t                   fw_image[SBV_OTA_SLOT_MAX_SIZE];
  uint32_t                  current_rcv_image_size;

  uint8_t                   is_update_enable;
  uint8_t                   is_updating;

  uint16_t                  seq_num;
  uint16_t                  peer_seq_num;

  sbv_rtos_queue_handle_t   slave_rx_installer_tx_queue;
  sbv_rtos_queue_handle_t   slave_tx_installer_rx_queue;
} sbv_ota_msg_slave_handler_t;

void
sbv_ota_slave_fsm_init (void* param);

#endif /* SBV_OTA_SLAVE_FSM_T */