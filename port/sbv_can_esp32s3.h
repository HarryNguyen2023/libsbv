#ifndef __SBV_CAN_ESP32S3_H__
#define __SBV_CAN_ESP32S3_H__

#include "sbv.h"
#include "sbv_rtos.h"

#ifdef ESP32xx_IDF
#include "driver/gpio.h"
#include "driver/twai.h"

#define SBV_CAN_STD_ID_NODE_ID          (2)
#define SBV_CAN_STD_ID_FILTER_ID_OFFSET (21)

typedef twai_message_t  sbv_can_tx_pkt_t;
typedef twai_message_t  sbv_can_rx_pkt_t;

typedef struct sbv_can_instance_t
{
    uint8_t                 can_active;
    uint8_t                 can_reg_callback;
    sbv_can_rx_pkt_t        can_rx_packet;
    void*                   can_handle;
    sbv_rtos_task_handle_t  can_rx_notify_task;
}sbv_can_instance_t;

void
sbv_can_esp32s3_init(void* can_handle);
int
sbv_can_esp32s3_send_data(sbv_can_msg_type_t msg_type, uint8_t *data, uint8_t length);
uint8_t *
sbv_can_esp32s3_rcv_data(uint8_t *length, uint16_t *std_id);
uint32_t
sbv_can_esp32s3_std_id_get (uint32_t msg_id);
#endif /* ESP32xx_IDF */
#endif /* __SBV_CAN_ESP32S3_H__ */