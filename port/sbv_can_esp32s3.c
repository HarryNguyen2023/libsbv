#include <string.h>
#include "sbv_rtos.h"
#include "sbv_can.h"
#include "sbv_can_esp32s3.h"

#ifdef ESP32xx_IDF

sbv_can_instance_t sbv_can_instance;

extern sbv_rtos_mutex_t SBV_CAN_RX_BUFFER_MUTEX;
extern sbv_rtos_mutex_t SBV_CAN_TX_BUFFER_MUTEX;

twai_general_config_t   g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_19, GPIO_NUM_20, TWAI_MODE_NORMAL);
twai_filter_config_t    filter_config;
twai_timing_config_t    time_config = TWAI_TIMING_CONFIG_500KBITS();

void
sbv_can_esp32s3_filter_init(void)
{
    memset (&filter_config, 0, sizeof (twai_filter_config_t));
    filter_config.acceptance_code   = (SBV_CAN_STD_ID_FILTER_ID << 21);
    filter_config.acceptance_mask   = ~(SBV_CAN_STD_ID_MASK << 21);
    filter_config.single_filter     = true;
}

void
sbv_can_esp32s3_init(void* can_handle)
{
    /*Intiiate CAN_RX filtering*/
    sbv_can_esp32s3_filter_init();

    memset (&sbv_can_instance, 0, sizeof (sbv_can_instance_t));
    sbv_can_instance.can_active          = SBV_TRUE;
    sbv_can_instance.can_rx_notify_task  = NULL;
    sbv_can_instance.can_handle          = can_handle;
    sbv_can_instance.can_reg_callback    = SBV_FALSE;

    /* Create the mutex for the CAN TX and RX FIFO */
    sbv_rtos_mutex_create(SBV_CAN_RX_BUFFER_MUTEX);
    sbv_rtos_mutex_create(SBV_CAN_TX_BUFFER_MUTEX);

    twai_driver_install(&g_config, &time_config, &filter_config);
    twai_start();

    return;
}

static int
sbv_can_esp32s3_send_pkt(sbv_can_tx_pkt_t *can_pkt)
{
    int ret = SBV_OK;

    if(! can_pkt)
        return SBV_ERROR;

    ret = twai_transmit(can_pkt, sbv_rtos_ms_to_tick(SBV_CAN_TX_TIMEOUT));

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
sbv_can_esp32s3_std_id_get (uint32_t msg_id)
{
    return ((SBV_CAN_STD_ID_BASE << SBV_CAN_STD_ID_BASE_OFFSET)
            | (msg_id << SBV_CAN_STD_ID_OFFSET)
            | (SBV_CAN_STD_ID_NODE_ID << SBV_CAN_STD_ID_NODE_ID_OFFSET));
}

static int
sbv_can_esp32s3_header_format(sbv_can_tx_pkt_t *can_pkt, sbv_can_msg_type_t msg_type,
                              uint8_t *data, uint8_t length)
{
    uint32_t std_id;
    int sent_bytes;

    if(!can_pkt || !data || !length)
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

    can_pkt->identifier         = sbv_can_esp32s3_std_id_get (std_id);
    can_pkt->extd               = SBV_FALSE;
    can_pkt->rtr                = SBV_FALSE;

    if(length <= SBV_CAN_DATA_MAX_SIZE)
    {
        can_pkt->data_length_code   = (length & 0xF);
        memcpy(can_pkt->data, data, length);
        sent_bytes                  = length;
    }
    else
    {
        can_pkt->data_length_code   = SBV_CAN_DATA_MAX_SIZE;
        memcpy(can_pkt->data, data, SBV_CAN_DATA_MAX_SIZE);
        sent_bytes                  = SBV_CAN_DATA_MAX_SIZE;
    }

    return sent_bytes;
}

int
sbv_can_esp32s3_send_data(sbv_can_msg_type_t msg_type, uint8_t *data, uint8_t length)
{
    int ret = SBV_OK, total_tx_bytes = 0, cur_tx_bytes = 0;
    sbv_can_tx_pkt_t can_tx_pkt;

    if(!data || !length
        || !(sbv_can_instance.can_active))
        return SBV_ERROR;

    SBV_CAN_TX_BUFFER_MUTEX_LOCK;

    while (total_tx_bytes < length)
    {
        cur_tx_bytes = sbv_can_esp32s3_header_format (&can_tx_pkt, msg_type, data + total_tx_bytes, length - total_tx_bytes);
        ret = sbv_can_esp32s3_send_pkt (&can_tx_pkt);
        if (ret != SBV_OK)
            continue;
        total_tx_bytes += cur_tx_bytes;
    }

    SBV_CAN_TX_BUFFER_MUTEX_UNLOCK;

    return total_tx_bytes;
}

uint8_t *
sbv_can_esp32s3_rcv_data(uint8_t *length, uint16_t *std_id)
{
    sbv_rtos_tick_type_t tick_to_wait;
    sbv_can_rx_pkt_t can_rx_pkt;

    tick_to_wait = SBV_CAN_RX_TIMEOUT;

    SBV_CAN_RX_BUFFER_MUTEX_LOCK;

    memset(&(sbv_can_instance.can_rx_packet), 0, sizeof(sbv_can_rx_pkt_t));

    twai_receive(&(sbv_can_instance.can_rx_packet), tick_to_wait);

    *length = (sbv_can_instance.can_rx_packet).data_length_code;
    *std_id = (sbv_can_instance.can_rx_packet).identifier;

    SBV_CAN_RX_BUFFER_MUTEX_UNLOCK;

    /* Avoid loopback CAN packet self-originate */
    if((*std_id & 0x1F) == SBV_CAN_STD_ID_NODE_ID)
    {
        *length = 0;
        return NULL;
    }

    return (sbv_can_instance.can_rx_packet).data;
}
#endif /* ESP32xx_IDF */