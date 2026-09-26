#ifndef __SBV_CAN_ESP32S3_H__
#define __SBV_CAN_ESP32S3_H__

#ifdef ESP32xx_IDF
#include "driver/twai.h"

#define SBV_CAN_DATA_MAX_SIZE           (8)

#define SBV_CAN_STD_ID_NODE_ID          (2)
#define SBV_CAN_STD_ID_FILTER_ID_OFFSET (21)

#define SBV_CAN_MAX_CHANNEL             (1)
#define SBV_CAN_RCV_BUFFER_SIZE         (64)

typedef twai_message_t      sbv_can_tx_pkt_t;
typedef twai_message_t      sbv_can_rx_pkt_t;
typedef uint8_t             sbv_can_handle_t;

typedef struct sbv_can_instance_t
{
    sbv_cqbuff*             can_rcv_buf;
}sbv_can_instance_t;

void
sbv_can_esp32s3_init(sbv_can_instance_t *can_instance,
                     void *can_handle);
int
sbv_can_esp32s3_send_data(sbv_can_instance_t *can_instance,
                          sbv_can_msg_type_t msg_type,
                          uint8_t *data, uint16_t length);
uint16_t
sbv_can_esp32s3_rcv_data (sbv_can_instance_t *can_instance,
                          uint8_t *rcv_buffer, uint16_t buffer_length,
                          uint16_t rcv_timeout_ms);
uint32_t
sbv_can_esp32s3_std_id_get (uint32_t msg_id);
#endif /* ESP32xx_IDF */
#endif /* __SBV_CAN_ESP32S3_H__ */