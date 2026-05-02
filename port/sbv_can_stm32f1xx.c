#include <string.h>
#include "sbv_rtos.h"
#include "sbv_can.h"
#include "sbv_can_stm32f1xx.h"

#ifdef STM32F1xx

#define sbv_can_stm32f1xx_rx_hw_callback \
        HAL_CAN_RxFifo1MsgPendingCallback

sbv_can_instance_t sbv_can_instance;

extern sbv_rtos_mutex_t SBV_CAN_RX_BUFFER_MUTEX;
extern sbv_rtos_mutex_t SBV_CAN_TX_BUFFER_MUTEX;

void
sbv_can_stm32f1xx_filter_init(sbv_can_handle_t *can_handle)
{
    CAN_FilterTypeDef canfilterconfig;

    if (! can_handle)
        return;

    memset (&canfilterconfig, 0, sizeof (CAN_FilterTypeDef));
    canfilterconfig.FilterActivation        = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank              = 10;
    canfilterconfig.FilterFIFOAssignment    = CAN_FILTER_FIFO1;
    canfilterconfig.FilterIdHigh            = SBV_CAN_STD_ID_FILTER_ID << SBV_CAN_STD_ID_FILTER_ID_OFFSET;
    canfilterconfig.FilterIdLow             = 0x0000;
    canfilterconfig.FilterMaskIdHigh        = SBV_CAN_STD_ID_MASK << SBV_CAN_STD_ID_FILTER_ID_OFFSET;
    canfilterconfig.FilterMaskIdLow         = 0x0000;
    canfilterconfig.FilterMode              = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale             = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank    = 0;

    HAL_CAN_ConfigFilter(can_handle, &canfilterconfig);
}

static int
sbv_can_stm32f1xx_callback_register(sbv_can_handle_t *can_handle)
{
    int ret = SBV_OK;

    if (! can_handle)
        return SBV_ERROR;

    ret = HAL_CAN_ActivateNotification(can_handle, CAN_IT_RX_FIFO1_MSG_PENDING);

    return ret;
}

static int
sbv_can_stm32f1xx_callback_deregister(sbv_can_handle_t *can_handle)
{
    int ret = SBV_OK;

    if (! can_handle)
        return SBV_ERROR;

    ret = HAL_CAN_DeactivateNotification(can_handle, CAN_IT_RX_FIFO1_MSG_PENDING);

    return ret;
}

void
sbv_can_stm32f1xx_init(sbv_can_handle_t *can_handle)
{
    if (! can_handle)
        return;

    /*Intiiate CAN_RX filtering*/
    sbv_can_stm32f1xx_filter_init (can_handle);

    memset (&sbv_can_instance, 0, sizeof (sbv_can_instance_t));
    sbv_can_instance.can_active          = SBV_TRUE;
    sbv_can_instance.can_rx_notify_task  = NULL;
    sbv_can_instance.can_handle          = can_handle;
    sbv_can_instance.can_reg_callback    = SBV_FALSE;

    /* Create the mutex for the CAN TX and RX FIFO */
    sbv_rtos_mutex_create(SBV_CAN_RX_BUFFER_MUTEX);
    sbv_rtos_mutex_create(SBV_CAN_TX_BUFFER_MUTEX);

    HAL_CAN_Start(can_handle);

    return;
}

static int
sbv_can_stm32f1xx_send_pkt(sbv_can_handle_t *can_handle, sbv_can_tx_pkt_t *can_pkt)
{
    int ret = SBV_OK;

    if(! can_pkt)
        return 0;

    ret = HAL_CAN_AddTxMessage (can_handle, &(can_pkt->sbv_can_header),
                                can_pkt->sbv_can_data, &(can_pkt->sbv_can_mailbox));

    return ret;
}

/*
 * The STD ID of CAN frame on SBV is comprised of the following components
 * Thus, each node must filter the first 6-bit of the message STD ID to ensure
 * receive the desired message of the system, as well as avoid loopback messages
 *  _____________________________
 * |         |        |         |
 * | Base ID | MSG ID | Node ID |
 * |_________|________|_________|
 *    3-bit    3-bit     5-bit   
 */
uint32_t
sbv_can_stm32f1xx_std_id_get (uint32_t msg_id)
{
    return ((SBV_CAN_STD_ID_BASE << SBV_CAN_STD_ID_BASE_OFFSET)
            | (msg_id << SBV_CAN_STD_ID_OFFSET)
            | (SBV_CAN_STD_ID_NODE_ID << SBV_CAN_STD_ID_NODE_ID_OFFSET));
}

static int
sbv_can_stm32f1xx_header_format (sbv_can_tx_pkt_t *can_pkt,
                                sbv_can_msg_type_t msg_type,
                                uint8_t *data, uint8_t length)
{
    uint32_t std_id;
    int sent_bytes;

    if(!can_pkt || ! length)
        return 0;

    memset(can_pkt, 0, sizeof(sbv_can_tx_pkt_t));

    /* Fill the CAN header */
    switch (msg_type)
    {
    case SBV_CAN_MSG_TUNNING:
        std_id = SBV_CAN_STD_ID_TUNNING;
        break;
    case SBV_CAN_MSG_COMMAND:
        std_id = SBV_CAN_STD_ID_COMMAND;
        break;
    case SBV_CAN_MSG_LOGGING:
        std_id = SBV_CAN_STD_ID_LOGGING;
        break;
    case SBV_CAN_MSG_OTA:
        std_id = SBV_CAN_STD_ID_OTA;
        break;

    default:
        return 0;
    }

    can_pkt->sbv_can_header.StdId   = sbv_can_stm32f1xx_std_id_get (std_id);
    can_pkt->sbv_can_header.IDE     = CAN_ID_STD;
    can_pkt->sbv_can_header.RTR     = CAN_RTR_DATA;

    if(length <= SBV_CAN_DATA_MAX_SIZE)
    {
        can_pkt->sbv_can_header.DLC = (length & 0xF);
        memcpy(can_pkt->sbv_can_data, data, length);
        sent_bytes                  = length;
    }
    else
    {
        can_pkt->sbv_can_header.DLC = SBV_CAN_DATA_MAX_SIZE;
        memcpy(can_pkt->sbv_can_data, data, SBV_CAN_DATA_MAX_SIZE);
        sent_bytes                  = SBV_CAN_DATA_MAX_SIZE;
    }

    return sent_bytes;
}

int
sbv_can_stm32f1xx_send_data(sbv_can_msg_type_t msg_type, uint8_t *data, uint8_t length)
{
    int ret = SBV_OK, total_tx_bytes = 0, cur_tx_bytes = 0;
    sbv_can_tx_pkt_t can_tx_pkt;

    if(!data || !length
        || !(sbv_can_instance.can_active))
        return SBV_ERROR;

    SBV_CAN_TX_BUFFER_MUTEX_LOCK;

    while (total_tx_bytes < length)
    {
        cur_tx_bytes = sbv_can_stm32f1xx_header_format(&can_tx_pkt, msg_type, data + total_tx_bytes, length - total_tx_bytes);
        ret = sbv_can_stm32f1xx_send_pkt(sbv_can_instance.can_handle, &can_tx_pkt);
        if (ret != SBV_OK)
            continue;
        total_tx_bytes += cur_tx_bytes;
    }

    SBV_CAN_TX_BUFFER_MUTEX_UNLOCK;

    return total_tx_bytes;
}

void
sbv_can_stm32f1xx_rx_hw_callback(sbv_can_handle_t *hcan)
{
    sbv_rtos_base_type_t xHigherPriorityTaskWoken = SBV_RTOS_FALSE;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &(sbv_can_instance.can_rx_packet.sbv_can_header),
                         sbv_can_instance.can_rx_packet.sbv_can_data);

    if(sbv_can_instance.can_rx_notify_task != NULL)
    {
        sbv_rtos_notify_give_fromISR(sbv_can_instance.can_rx_notify_task, &xHigherPriorityTaskWoken);
        sbv_rtos_port_yield_fromISR(xHigherPriorityTaskWoken);
    }
}

uint8_t*
sbv_can_stm32f1xx_rcv_data (uint8_t *length, uint16_t *std_id)
{
    sbv_rtos_tick_type_t tick_to_wait;

    tick_to_wait = SBV_CAN_RX_TIMEOUT;

    SBV_CAN_RX_BUFFER_MUTEX_LOCK;

    sbv_can_instance.can_rx_notify_task = sbv_rtos_get_current_task_handle();

    if (sbv_can_instance.can_reg_callback == SBV_FALSE)
    {
        sbv_can_stm32f1xx_callback_register (sbv_can_instance.can_handle);
        sbv_can_instance.can_reg_callback = SBV_TRUE;
    }

    sbv_rtos_notify_take(SBV_RTOS_TRUE, tick_to_wait);
    sbv_can_instance.can_rx_notify_task = NULL;

    *length = (sbv_can_instance.can_rx_packet.sbv_can_header.DLC & 0xFF);
    *std_id = (sbv_can_instance.can_rx_packet.sbv_can_header.StdId & 0xFFFF);

    SBV_CAN_RX_BUFFER_MUTEX_UNLOCK;

    /* Avoid loopback CAN packet self-originate */
    if((*std_id & 0x1F) == SBV_CAN_STD_ID_NODE_ID)
    {
        *length = 0;
        return NULL;
    }

    return sbv_can_instance.can_rx_packet.sbv_can_data;
}
#endif  /*STM32F1xx*/