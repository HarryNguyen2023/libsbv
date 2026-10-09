#ifndef SBV_OTA_MSG_FSM_HELPER_H
#define SBV_OTA_MSG_FSM_HELPER_H

int
sbv_ota_msg_fsm_handle_cmd(sbv_cqbuff *pkt_rcv_queue, uint16_t* peer_seq_num,
                           sbv_ota_cmd_t cmd_type, uint32_t timeout_ms);

int
sbv_ota_msg_fsm_handle_header(sbv_cqbuff *pkt_rcv_queue,
                              uint16_t* peer_seq_num, uint32_t timeout_ms,
                              sbv_ota_fw_metadata_t *out_fw_metadata);
int
sbv_ota_msg_fsm_handle_data(sbv_cqbuff *pkt_rcv_queue, uint16_t* peer_seq_num,
                            uint32_t timeout_ms, uint8_t *out_fw_image,
                            uint32_t* total_rcv_image_length,
                            uint32_t expected_total_img_length);

#endif /* SBV_OTA_MSG_FSM_HELPER_H */ 