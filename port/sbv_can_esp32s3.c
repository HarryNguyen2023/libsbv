#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_log.h"
#include "sbv_rtos.h"
#include "sbv_gpio.h"
#include "sbv_cqbuff.h"
#include "sbv_can.h"
#include "sbv_can_esp32s3.h"

#ifdef ESP32xx_IDF

twai_general_config_t   gen_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
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
sbv_can_esp32s3_init(sbv_can_instance_t *can_instance,
                     void *can_handle)
{
    /*Intiiate CAN_RX filtering*/
    // sbv_can_esp32s3_filter_init();

    memset (can_instance, 0, sizeof (sbv_can_instance_t));
    can_instance->can_rcv_buf = sbv_cqbuff_create(SBV_CAN_RCV_BUFFER_SIZE, 1);

    twai_driver_install(&gen_config, &time_config, &filter_config);
    twai_start();

    return;
}

static int
sbv_can_esp32s3_send_pkt(const sbv_can_tx_pkt_t *can_pkt)
{
    int ret = SBV_OK;

    if(! can_pkt)
        return SBV_ERROR;

    ret = twai_transmit(can_pkt, sbv_rtos_ms_to_tick(0));

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
sbv_can_esp32s3_header_format(sbv_can_tx_pkt_t *can_pkt,
                              sbv_can_msg_type_t msg_type,
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
sbv_can_esp32s3_send_data(sbv_can_instance_t *can_instance,
                          sbv_can_msg_type_t msg_type,
                          uint8_t *data, uint16_t length)
{
    int ret = SBV_OK;
    uint8_t try_num = 0;
    uint16_t total_tx_bytes = 0, cur_tx_bytes = 0;
    sbv_can_tx_pkt_t can_tx_pkt;

    if(! can_instance || ! data || (length == 0)) {
        LOG_ERROR ("Invalid input, skipping sending CAN data");
        return SBV_ERROR;
    }

    while (total_tx_bytes < length)
    {
        cur_tx_bytes = sbv_can_esp32s3_header_format(&can_tx_pkt, msg_type,
                                                    data + total_tx_bytes,
                                                    length - total_tx_bytes);
        ret = sbv_can_esp32s3_send_pkt (&can_tx_pkt);
        if (ret != SBV_OK) {
            LOG_ERROR ("Failed to send CAN data, retry num=%u", ++try_num);
            if (try_num >= SBV_CAN_MAX_WRITE_RETRY)
            {
                break;
            }

            continue;
        }
        total_tx_bytes += cur_tx_bytes;
    }

    LOG_DEBUG ("Write %u bytes via CAN TX, retry num=%u", total_tx_bytes, try_num);

    return total_tx_bytes;
}

uint16_t
sbv_can_esp32s3_rcv_data (sbv_can_instance_t *can_instance,
                          uint8_t *rcv_buffer, uint16_t buffer_length,
                          uint16_t rcv_timeout_ms)
{
    int ret;
    uint16_t rx_buffer_size;
    sbv_rtos_tick_type_t tick_to_wait;
    sbv_can_rx_pkt_t can_rx_pkt;

    if (! can_instance || ! rcv_buffer || buffer_length == 0) {
        LOG_ERROR ("Invallid input, skipping receive CAN data");
        return 0;
    }

    tick_to_wait = sbv_rtos_ms_to_tick(rcv_timeout_ms);

    memset(&can_rx_pkt, 0, sizeof(sbv_can_rx_pkt_t));

    ret = twai_receive(&can_rx_pkt, tick_to_wait);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to rcv CAN data, ret=%d", ret);
        return 0;
    }

    if (can_instance->can_rcv_buf)
    {
        sbv_cqbuff_write (can_instance->can_rcv_buf,
                          (unsigned char *)&can_rx_pkt, sizeof(sbv_can_rx_pkt_t));
    }

    rx_buffer_size = sbv_cqbuff_get_size (can_instance->can_rcv_buf);
    if (rx_buffer_size == 0)
        return 0;

    LOG_DEBUG ("Received %u bytes via CAN RX", rx_buffer_size);

    rx_buffer_size = (rx_buffer_size < buffer_length) ? rx_buffer_size : buffer_length;
    rx_buffer_size = sbv_cqbuff_read (can_instance->can_rcv_buf,
                                      rcv_buffer, rx_buffer_size);

    return rx_buffer_size;
}
#endif /* ESP32xx_IDF */