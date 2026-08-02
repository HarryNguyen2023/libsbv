#include <string.h>
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_can.h"
#include "sbv_can_stm32f1xx.h"

#ifdef STM32F1xx

struct sbv_can_instances_list_t sbv_can_instances_list = {0};

#define sbv_can_stm32f1xx_rx_hw_callback \
        HAL_CAN_RxFifo1MsgPendingCallback

/* Return the can instance that owns the given HAL can handle. */
static sbv_can_instance_t*
sbv_can_stm32f1xx_get_instance_by_handle (sbv_can_handle_t* can_handle)
{
    if (! can_handle)
        return NULL;

    for (uint8_t i = 0; i < SBV_CAN_MAX_CHANNEL; ++i)
    {
        if (sbv_can_instances_list.list[i]
            && sbv_can_instances_list.list[i]->can_handle == can_handle)
        {
           return sbv_can_instances_list.list[i];
        }
    }

    return NULL;
}

/* Register a new can instance in the internal instance list. */
static int
sbv_can_stm32f1xx_add_instance_to_list (sbv_can_instance_t *can_instance)
{
    if (! can_instance)
        return SBV_ERROR;

    for (uint8_t i = 0; i < SBV_CAN_MAX_CHANNEL; ++i)
    {
        if (sbv_can_instances_list.list[i] == NULL)
        {
           sbv_can_instances_list.list[i] =  can_instance;
           return SBV_OK;
        }
    }

    return SBV_ERROR;
}

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
sbv_can_stm32f1xx_init(sbv_can_instance_t *can_instance,
                       sbv_can_handle_t *can_handle)
{
    if (! can_instance || ! can_handle)
        return;

    if (sbv_can_stm32f1xx_add_instance_to_list (can_instance) != SBV_OK)
    {
        /* LOG */
        return;
    }

    /*Intiiate CAN_RX filtering*/
    sbv_can_stm32f1xx_filter_init (can_handle);

    memset (can_instance, 0, sizeof (sbv_can_instance_t));
    can_instance->can_active          = SBV_TRUE;
    can_instance->can_rx_notify_task  = NULL;
    can_instance->can_handle          = can_handle;
    can_instance->can_reg_callback    = SBV_FALSE;
    can_instance->can_rcv_buf         = sbv_cqbuff_create(SBV_CAN_RCV_BUFFER_SIZE, 1);

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
                                uint8_t *data, uint16_t length)
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
sbv_can_stm32f1xx_send_data(sbv_can_instance_t *can_instance,
                            sbv_can_msg_type_t msg_type,
                            uint8_t *data, uint16_t length)
{
    int ret = SBV_OK;
    uint8_t try_num = 0;
    uint16_t total_tx_bytes = 0, cur_tx_bytes = 0;
    sbv_can_tx_pkt_t can_tx_pkt;

    if(! can_instance || !data || (length == 0)
        || !(can_instance->can_active))
        return SBV_ERROR;

    while (total_tx_bytes < length)
    {
        cur_tx_bytes = sbv_can_stm32f1xx_header_format(&can_tx_pkt, msg_type,
                                                        data + total_tx_bytes,
                                                        length - total_tx_bytes);
        ret = sbv_can_stm32f1xx_send_pkt(can_instance->can_handle, &can_tx_pkt);
        if (ret != SBV_OK)
        {
            // LOG
            if (try_num++ >= SBV_CAN_MAX_WRITE_RETRY)
            {
                // LOG
                break;
            }
        }
        total_tx_bytes += cur_tx_bytes;
    }

    return total_tx_bytes;
}

void
sbv_can_stm32f1xx_rx_hw_callback(sbv_can_handle_t *can_hanlde)
{
    sbv_can_instance_t *can_instance = NULL;
    sbv_can_rx_pkt_t can_rx_packet;
    sbv_rtos_base_type_t xHigherPriorityTaskWoken = SBV_RTOS_FALSE;

    can_instance = sbv_can_stm32f1xx_get_instance_by_handle (can_hanlde);
    if (can_instance == NULL)
    {
        // LOG
        return;
    }

    HAL_CAN_GetRxMessage(can_hanlde, CAN_RX_FIFO1,
                         &(can_rx_packet.sbv_can_header),
                         can_rx_packet.sbv_can_data);

    if (can_instance->can_active
        && can_instance->can_rcv_buf)
    {
        sbv_cqbuff_write (can_instance->can_rcv_buf, &can_rx_packet, sizeof(sbv_can_rx_pkt_t));
    }

    if(can_instance->can_rx_notify_task != NULL)
    {
        sbv_rtos_notify_give_fromISR(can_instance->can_rx_notify_task,
                                     &xHigherPriorityTaskWoken);
        sbv_rtos_port_yield_fromISR(xHigherPriorityTaskWoken);
    }
}

uint16_t
sbv_can_stm32f1xx_rcv_data (sbv_can_instance_t *can_instance,
                            uint8_t *rcv_buffer, uint16_t buffer_length,
                            uint16_t rcv_timeout_ms)
{
    uint16_t rx_buffer_size;
    sbv_rtos_tick_type_t tick_to_wait;

    tick_to_wait = sbv_rtos_ms_to_tick(rcv_timeout_ms);

    if (can_instance->can_reg_callback == SBV_FALSE)
    {
        sbv_can_stm32f1xx_callback_register (can_instance->can_handle);
        can_instance->can_reg_callback = SBV_TRUE;
    }

    can_instance->can_rx_notify_task = sbv_rtos_get_current_task_handle();

    sbv_rtos_notify_take(SBV_RTOS_TRUE, tick_to_wait);

    can_instance->can_rx_notify_task = NULL;

    rx_buffer_size = sbv_cqbuff_get_size (can_instance->can_rcv_buf);
    if (rx_buffer_size == 0)
        return 0;

    rx_buffer_size = (rx_buffer_size < buffer_length) ? rx_buffer_size : buffer_length;
    rx_buffer_size = sbv_cqbuff_read (can_instance->can_rcv_buf,
                                      rcv_buffer, rx_buffer_size);

    return rx_buffer_size;
}
#endif  /*STM32F1xx*/