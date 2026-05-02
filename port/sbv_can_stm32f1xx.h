#ifndef __SBV_CAN_STM32F1XX_H__
#define __SBV_CAN_STM32F1XX_H__

#include "sbv.h"
#include "sbv_rtos.h"

#ifdef STM32F1xx
#include "stm32f1xx_hal_can.h"

#define SBV_CAN_DATA_MAX_SIZE           (8)
#define SBV_CAN_STD_ID_NODE_ID          (1)
#define SBV_CAN_STD_ID_FILTER_ID_OFFSET (5)

typedef CAN_HandleTypeDef       sbv_can_handle_t;
typedef CAN_TxHeaderTypeDef     sbv_can_tx_header_t;

typedef CAN_RxHeaderTypeDef     sbv_can_rx_header_t;

typedef struct sbv_can_tx_pkt_t
{
    sbv_can_tx_header_t     sbv_can_header;
    uint8_t                 sbv_can_data[SBV_CAN_DATA_MAX_SIZE];
    uint32_t                sbv_can_mailbox;
} sbv_can_tx_pkt_t;

typedef struct sbv_can_rx_pkt_t
{
    sbv_can_rx_header_t     sbv_can_header;
    uint8_t                 sbv_can_data[SBV_CAN_DATA_MAX_SIZE];
} sbv_can_rx_pkt_t;

typedef struct sbv_can_instance_t
{
    uint8_t                 can_active;
    uint8_t                 can_reg_callback;
    sbv_can_rx_pkt_t        can_rx_packet;
    sbv_can_handle_t*       can_handle;
    sbv_rtos_task_handle_t  can_rx_notify_task;
}sbv_can_instance_t;

void
sbv_can_stm32f1xx_init(sbv_can_handle_t *can_handle);
int
sbv_can_stm32f1xx_send_data(sbv_can_msg_type_t msg_type, uint8_t *data, uint8_t length);
uint8_t*
sbv_can_stm32f1xx_rcv_data (uint8_t *length, uint16_t *std_id);
uint32_t
sbv_can_stm32f1xx_std_id_get (uint32_t msg_id);
#endif /* STM32F1xx */
#endif /* __SBV_CAN_STM32F1XX_H__ */