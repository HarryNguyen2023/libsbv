#ifndef SBV_OTA_MASTER_FSM_H
#define SBV_OTA_MASTER_FSM_H

typedef struct sbv_ota_msg_master_handler_t
{
  sbv_cqbuff*             data_queue;

  sbv_ota_state_t         state;
  sbv_ota_state_t         next_state;

  uint8_t                 is_updating;
  uint8_t                 max_retry;
  uint8_t                 is_ack;
  uint8_t                 is_abort;

  uint16_t                seq_num;
  uint16_t                peer_seq_num;

  sbv_rtos_mutex_t        mu;
  sbv_rtos_queue_handle_t rx_queue;
} sbv_ota_msg_master_handler_t;

void
sbv_ota_master_fsm_init (void);

#endif /* SBV_OTA_MASTER_FSM_H */