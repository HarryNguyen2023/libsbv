#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_uart.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_uart.h"
#include "sbv_can.h"
#include "sbv_ota.h"
#include "sbv_ota_msg.h"

#ifdef SBV_HW_CRC
#ifdef STM32F1xx
extern CRC_HandleTypeDef hcrc;
#elif defined ESP32xx_IDF
#include "esp_crc.h"
#endif /*STM32F1xx*/
#endif /* SBV_HW_CRC */

#define SBV_OTA_MSG_TX_TIMEOUT_MS      (100)
#define SBV_OTA_MSG_RX_TIMEOUT_MS      (100)

#define SBV_OTA_NEXT_STATE(NS,T,ACK) \
    ((T) == sbv_ota_msg_tx_instance.max_retry) ? SBV_OTA_STATE_IDLE : \
        (((ACK)!=SBV_TRUE) ? SBV_OTA_STATE_IDLE : NS)

sbv_ota_msg_hw_cb_t sbv_ota_msg_hw_cb = {
#ifdef SBV_OTA_CAN
    .sbv_ota_msg_send = sbv_can_send_data,
    .sbv_ota_reg_cb   = NULL,
    .sbv_ota_rcv_data = NULL,
#endif /* SBV_OTA_CAN */
#ifdef SBV_OTA_UART
    .sbv_ota_msg_send = sbv_uart_send_ota_data,
    .sbv_ota_reg_cb   = sbv_uart_register_rx_cb,
    .sbv_ota_rcv_data = sbv_uart_rx_rcv_data,
#endif /* SBV_OTA_UART */
};

void sbv_ota_state_idle (sbv_ota_state_t current_state, void *data);
void sbv_ota_state_start (sbv_ota_state_t current_state, void *data);
void sbv_ota_state_header (sbv_ota_state_t current_state, void *data);
void sbv_ota_state_data (sbv_ota_state_t current_state, void *data);
void sbv_ota_state_end (sbv_ota_state_t current_state, void *data);

struct sbv_ota_state_cb_t {
    sbv_ota_state_t next_state;
    void            (*state_func) (sbv_ota_state_t, void *);
};

struct sbv_ota_state_cb_t sbv_ota_states[SBV_OTA_STATE_MAX][SBV_OTA_STATE_MAX] = {
    {{SBV_OTA_STATE_IDLE,   sbv_ota_state_idle},
     {SBV_OTA_STATE_START,  sbv_ota_state_start},
     {SBV_OTA_STATE_HEADER, sbv_ota_state_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_state_idle},
     {SBV_OTA_STATE_END,    sbv_ota_state_idle}},
    
    {{SBV_OTA_STATE_IDLE,   sbv_ota_state_idle},
     {SBV_OTA_STATE_START,  sbv_ota_state_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_state_header},
     {SBV_OTA_STATE_DATA,   sbv_ota_state_idle},
     {SBV_OTA_STATE_END,    sbv_ota_state_idle}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_state_idle},
     {SBV_OTA_STATE_START,  sbv_ota_state_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_state_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_state_data},
     {SBV_OTA_STATE_END,    sbv_ota_state_idle}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_state_idle},
     {SBV_OTA_STATE_START,  sbv_ota_state_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_state_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_state_idle},
     {SBV_OTA_STATE_END,    sbv_ota_state_end}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_state_idle},
     {SBV_OTA_STATE_START,  sbv_ota_state_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_state_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_state_idle},
     {SBV_OTA_STATE_END,    sbv_ota_state_idle}},
};

sbv_ota_msg_rx_instance_t sbv_ota_msg_rx_instance = {0};
sbv_ota_msg_tx_instance_t sbv_ota_msg_tx_instance = {0};
uint8_t sbv_ota_msg_rcv_buffer[SBV_OTA_PACKET_MAX_SIZE];

extern sbv_event_group_handle_t sbv_ota_event_group;

static int
sbv_ota_msg_send (uint8_t *data, uint16_t length, uint16_t timeout_ms)
{
    if (sbv_ota_msg_hw_cb.sbv_ota_msg_send)
    {
        // When sending OTA msg via UART, the first parameters will be ignored
        return (sbv_ota_msg_hw_cb.sbv_ota_msg_send) (SBV_CAN_MSG_OTA, data, length, timeout_ms);
    }

    return 0;
}

static uint8_t*
sbv_ota_msg_rcv (uint16_t length, uint16_t timeout_ms)
{
    if (sbv_ota_msg_hw_cb.sbv_ota_rcv_data)
    {
        return (sbv_ota_msg_hw_cb.sbv_ota_rcv_data) (length, timeout_ms);
    }

    return 0;
}

uint32_t
sbv_ota_msg_crc_calculate (uint8_t *buffer, uint32_t buffer_length)
{
#ifdef SBV_HW_CRC
#ifdef STM32F1xx
    return HAL_CRC_Calculate(&hcrc, buffer, buffer_length);
#elif defined ESP32xx_IDF
    return esp_crc32_le(0xFFFFFFFF, (uint8_t *)buffer, buffer_length);
#endif /*STM32F1xx*/
#else
    uint32_t crc = 0xFFFFFFFF;

    if(!buffer || !buffer_length)
        return 0;

    for (uint32_t i = 0; i < buffer_length; i++)
    {
        crc ^= *(buffer + i);
        for (uint8_t j = 0; j < 32; j++)
        {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
#endif /*SBV_HW_CRC*/
}

int
sbv_ota_msg_send_resp (uint8_t resp_type)
{
    uint32_t pkt_crc;
	sbv_ota_resp_pkt_t resp_pkt;

    if((resp_type != SBV_OTA_ACK) && (resp_type != SBV_OTA_NACK))
        return -1;

    resp_pkt.sof 			= SBV_OTA_SOF;
    resp_pkt.packet_type 	= SBV_OTA_PACKET_TYPE_RESPONSE;
    resp_pkt.status 		= resp_type;
    resp_pkt.crc            = 0;
    resp_pkt.eof			= SBV_OTA_EOF;

	pkt_crc = sbv_ota_msg_crc_calculate((uint8_t *)&resp_pkt, sizeof(sbv_ota_resp_pkt_t));
    resp_pkt.crc = pkt_crc;

    return sbv_ota_msg_send((uint8_t *)&resp_pkt, sizeof(sbv_ota_resp_pkt_t), SBV_OTA_MSG_TX_TIMEOUT_MS);
}

int
sbv_ota_msg_send_cmd (sbv_ota_cmd_t cmd_type)
{
    uint32_t pkt_crc;
	sbv_ota_cmd_pkt_t cmd_pkt;

    cmd_pkt.sof 			= SBV_OTA_SOF;
    cmd_pkt.packet_type 	= SBV_OTA_PACKET_TYPE_CMD;
    cmd_pkt.cmd 		    = cmd_type;
    cmd_pkt.crc             = 0;
    cmd_pkt.eof			    = SBV_OTA_EOF;

	pkt_crc = sbv_ota_msg_crc_calculate((uint8_t *)&cmd_pkt, sizeof(sbv_ota_cmd_pkt_t));
    cmd_pkt.crc = pkt_crc;

    return sbv_ota_msg_send((uint8_t *)&cmd_pkt, sizeof(sbv_ota_cmd_pkt_t), SBV_OTA_MSG_TX_TIMEOUT_MS);
}

int
sbv_ota_msg_send_data_header(uint8_t *data, sbv_ota_fw_metadata_t* data_info)
{
    sbv_ota_header_pkt_t header_pkt;
    uint32_t data_crc, pkt_crc;

    if(! data || ! data_info)
        return -1;

    data_crc = sbv_ota_msg_crc_calculate((uint8_t *)data, data_info->fw_size);
    if (data_crc != data_info->fw_crc)
    {
        /* LOG */
        return -1;
    }

    header_pkt.sof                      = SBV_OTA_SOF;
    header_pkt.packet_type              = SBV_OTA_PACKET_TYPE_HEADER;
    memcpy (&header_pkt.data_info, data_info, sizeof (sbv_ota_fw_metadata_t));
    header_pkt.crc                      = 0;
    header_pkt.eof                      = SBV_OTA_EOF;

    pkt_crc = sbv_ota_msg_crc_calculate((uint8_t *)&header_pkt, sizeof(sbv_ota_header_pkt_t));
    header_pkt.crc = pkt_crc;

    return sbv_ota_msg_send((uint8_t *)&header_pkt, sizeof(sbv_ota_header_pkt_t), SBV_OTA_MSG_TX_TIMEOUT_MS);
}

int 
sbv_ota_msg_send_data_frame(uint8_t *data, uint32_t data_length)
{
    sbv_ota_data_pkt_t *data_pkt = NULL;
    int32_t total_length = data_length;
    uint32_t pkt_length;
    uint32_t pkt_data_length;
    int ret;

    if(! data || ! data_length)
        return -1;

    do
    {
        if(total_length > SBV_OTA_DATA_MAX_SIZE)
        {
            pkt_data_length = SBV_OTA_DATA_MAX_SIZE;
            total_length    -= SBV_OTA_DATA_MAX_SIZE;
        }
        else
        {
            pkt_data_length = total_length;
            total_length    = 0;
        }
        pkt_length = sizeof(sbv_ota_data_pkt_t) + pkt_data_length;
        data_pkt   = sbv_rtos_malloc(pkt_length);
        if(! data_pkt)
        {
            /* LOG */
            return -1;
        }

        data_pkt->sof           = SBV_OTA_SOF;
        data_pkt->packet_type   = SBV_OTA_PACKET_TYPE_DATA;
        data_pkt->crc           = 0;
        memcpy(data_pkt->data, data, pkt_data_length);
        data                    += pkt_data_length;
        data_pkt->crc           = sbv_ota_msg_crc_calculate((uint8_t *)data_pkt, pkt_length);

        ret = sbv_ota_msg_send((uint8_t *)data_pkt, pkt_length, SBV_OTA_MSG_TX_TIMEOUT_MS);
        if (ret < 0)
        {
            /* LOG */
            goto ERR;
        }

        if(data_pkt)
        {
            sbv_rtos_free(data_pkt);
            data_pkt = NULL;
        }
    } while (total_length > 0);

    return data_length;

ERR:
    if(data_pkt)
    {
        sbv_rtos_free(data_pkt);
        data_pkt = NULL;
    }
    return -1;
}

void sbv_ota_handle_state (sbv_ota_state_t current_state, sbv_ota_state_t next_state, void *data)
{
    struct sbv_ota_state_cb_t *state_cb;

    if (current_state == next_state
        || (current_state < SBV_OTA_STATE_IDLE || current_state >= SBV_OTA_STATE_MAX)
        || (next_state < SBV_OTA_STATE_IDLE || next_state >= SBV_OTA_STATE_MAX))
        return;

    if (next_state == SBV_OTA_STATE_IDLE)
    {
        sbv_ota_msg_tx_instance.tx_state    = SBV_OTA_STATE_IDLE;
        sbv_ota_msg_tx_instance.next_state  = SBV_OTA_STATE_IDLE;
        sbv_ota_msg_tx_instance.is_updating = SBV_FALSE;
        return;
    }

    state_cb = &sbv_ota_states[current_state][next_state];
    if (! state_cb || state_cb->next_state != next_state)
        return;

    sbv_ota_msg_tx_instance.tx_state = next_state;
    (*state_cb->state_func) (current_state, data);
}

void sbv_ota_state_idle (sbv_ota_state_t current_state, void *data)
{
    /* Do nothing */
    return;
}

void sbv_ota_state_start (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t *buff, retry_time;
    uint16_t data_length;

    if (current_state != SBV_OTA_STATE_IDLE)
    {
        sbv_ota_msg_tx_instance.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    retry_time = 0;
    while (retry_time < sbv_ota_msg_tx_instance.max_retry)
    {
        sbv_ota_msg_tx_instance.is_ack = SBV_FALSE;
        ret = sbv_ota_msg_send_cmd (SBV_OTA_CMD_START);
        if (ret < 0)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* This function will call the callback function for handling respose from peer */
        buff = sbv_ota_msg_rcv (&data_length, SBV_OTA_MSG_RX_TIMEOUT_MS);
        if (! buff || ! data_length
            || ! (sbv_ota_msg_tx_instance.is_ack))
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* LOG */
        break;
    }

    sbv_ota_msg_tx_instance.next_state = SBV_OTA_NEXT_STATE(SBV_OTA_STATE_HEADER,
                                                            retry_time,
                                                            sbv_ota_msg_tx_instance.is_ack);

    return;
}

void sbv_ota_state_header (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t *buff, retry_time, *images;
    uint16_t data_length;
    sbv_ota_fw_metadata_t *data_info;

    if (current_state != SBV_OTA_STATE_START)
    {
        sbv_ota_msg_tx_instance.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    // Read the images data from the filesystem

    retry_time = 0;
    while (retry_time < sbv_ota_msg_tx_instance.max_retry)
    {
        sbv_ota_msg_tx_instance.is_ack = SBV_FALSE;
        ret = sbv_ota_msg_send_data_header (images, data_info);
        if (ret < 0)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* This function will call the callback function for handling respose from peer */
        buff = sbv_ota_msg_rcv (&data_length, SBV_OTA_MSG_RX_TIMEOUT_MS);
        if (! buff || ! data_length
            || ! (sbv_ota_msg_tx_instance.is_ack))
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* LOG */
        break;
    }

    sbv_ota_msg_tx_instance.next_state = SBV_OTA_NEXT_STATE(SBV_OTA_STATE_DATA,
                                                            retry_time,
                                                            sbv_ota_msg_tx_instance.is_ack);

    return;
}

void sbv_ota_state_data (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t *buff, retry_time, *images;
    uint16_t data_length, image_length;

    if (current_state != SBV_OTA_STATE_HEADER)
    {
        sbv_ota_msg_tx_instance.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    // Read the images from the filesystem

    retry_time = 0;
    while (retry_time < sbv_ota_msg_tx_instance.max_retry)
    {
        sbv_ota_msg_tx_instance.is_ack = SBV_FALSE;
        ret = sbv_ota_msg_send_data_frame (images, image_length);
        if (ret < 0)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* This function will call the callback function for handling respose from peer */
        buff = sbv_ota_msg_rcv (&data_length, SBV_OTA_MSG_RX_TIMEOUT_MS);
        if (! buff || ! data_length
            || ! (sbv_ota_msg_tx_instance.is_ack))
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* LOG */
        break;
    }

    sbv_ota_msg_tx_instance.next_state = SBV_OTA_NEXT_STATE(SBV_OTA_STATE_END,
                                                            retry_time,
                                                            sbv_ota_msg_tx_instance.is_ack);

    return;
}

void sbv_ota_state_end (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t *buff, retry_time;
    uint16_t data_length;

    if (current_state != SBV_OTA_STATE_DATA)
    {
        sbv_ota_msg_tx_instance.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    retry_time = 0;
    while (retry_time < sbv_ota_msg_tx_instance.max_retry)
    {
        sbv_ota_msg_tx_instance.is_ack = SBV_FALSE;
        ret = sbv_ota_msg_send_cmd (SBV_OTA_CMD_END);
        if (ret < 0)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* This function will call the callback function for handling respose from peer */
        buff = sbv_ota_msg_rcv (&data_length, SBV_OTA_MSG_RX_TIMEOUT_MS);
        if (! buff || ! data_length
            || ! (sbv_ota_msg_tx_instance.is_ack))
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* LOG */
        break;
    }

    sbv_ota_msg_tx_instance.next_state = SBV_OTA_NEXT_STATE(SBV_OTA_STATE_IDLE,
                                                            retry_time,
                                                            sbv_ota_msg_tx_instance.is_ack);
    sbv_ota_msg_tx_instance.is_updating = SBV_FALSE;

    return;
}

int
sbv_ota_msg_rx_handle_cmd(uint8_t *data, uint32_t data_length)
{
    sbv_ota_cmd_pkt_t *cmd_pkt;
    uint32_t pkt_crc, new_crc;

    if(! data || ! data_length)
        return SBV_ERROR;

    if(data_length != sizeof(sbv_ota_cmd_pkt_t))
    {
        /* LOG */
        return SBV_ERROR;
    }

    cmd_pkt = (sbv_ota_cmd_pkt_t *)data;

    if((cmd_pkt->sof != SBV_OTA_SOF) || (cmd_pkt->eof != SBV_OTA_EOF))
        return SBV_ERROR;

    if(cmd_pkt->packet_type != SBV_OTA_PACKET_TYPE_CMD)
        return SBV_ERROR;

    pkt_crc         = cmd_pkt->crc;
    cmd_pkt->crc    = 0;
    new_crc         = sbv_ota_msg_crc_calculate((uint8_t *)cmd_pkt, data_length);
    if(pkt_crc != new_crc)
        return SBV_ERROR;

    if((sbv_ota_msg_rx_instance.rx_state == SBV_OTA_STATE_START)
        && (cmd_pkt->cmd == SBV_OTA_CMD_START))
        sbv_ota_msg_rx_instance.rx_state = SBV_OTA_STATE_HEADER;
    else if((sbv_ota_msg_rx_instance.rx_state == SBV_OTA_STATE_END)
            && (cmd_pkt->cmd == SBV_OTA_CMD_END))
    {
        sbv_ota_msg_rx_instance.rx_state = SBV_OTA_STATE_IDLE;
        /* Trigger to send report to the peer */
        sbv_rtos_event_group_set_bits(sbv_ota_event_group, SBV_OTA_RCV_ALL);
    }
    else
        return SBV_ERROR;

    return SBV_OK;
}

int
sbv_ota_msg_rx_handle_header(uint8_t *data, uint32_t data_length)
{
    sbv_ota_header_pkt_t header_pkt;
    uint32_t pkt_crc, new_crc;
    int ret;

    if(! data || ! data_length)
        return SBV_ERROR;

    sbv_rtos_mutex_lock(sbv_ota_msg_rx_instance.mutex);

    ret = sbv_cqbuff_write (sbv_ota_msg_rx_instance.rx_queue, data, data_length);
    if (ret <= 0)
    {
        /* LOG */
        goto ERR_EXIT;
    }

    if(sbv_cqbuff_get_size (sbv_ota_msg_rx_instance.rx_queue) < sizeof(sbv_ota_header_pkt_t))
    {
        sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);
        return SBV_BUSY;
    }

    sbv_cqbuff_read(sbv_ota_msg_rx_instance.rx_queue, &header_pkt, sizeof(sbv_ota_header_pkt_t));
    if((header_pkt.sof != SBV_OTA_SOF) || (header_pkt.eof != SBV_OTA_EOF))
        goto ERR_EXIT;

    if(header_pkt.packet_type != SBV_OTA_PACKET_TYPE_HEADER)
        goto ERR_EXIT;

    pkt_crc         = header_pkt.crc;
    header_pkt.crc  = 0;
    new_crc         = sbv_ota_msg_crc_calculate((uint8_t *)&header_pkt, sizeof(sbv_ota_header_pkt_t));
    if(pkt_crc != new_crc)
    {
        /* LOG */
        goto ERR_EXIT;
    }

    ret = sbv_cqbuff_write (sbv_ota_msg_rx_instance.rx_queue, &(header_pkt.data_info),
                            sizeof(sbv_ota_fw_metadata_t));
    if (ret <= 0)
    {
        /* LOG */
        goto ERR_EXIT;
    }

    sbv_ota_msg_rx_instance.data_size   = header_pkt.data_info.fw_size;
    sbv_ota_msg_rx_instance.rx_state    = SBV_OTA_STATE_DATA;

    sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);

    /* Trigger event to sbv_ota_update_fw_thread */
    sbv_rtos_event_group_set_bits(sbv_ota_event_group, SBV_OTA_RCV_METADATA);
    
    return SBV_OK;

ERR_EXIT:
    sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);
    return SBV_ERROR;
}

int
sbv_ota_msg_handle_data(uint8_t *data, uint32_t data_length)
{
    sbv_ota_data_pkt_t* data_pkt;
    uint32_t pkt_crc, new_crc;
    int ret, rcv_size;

    if(! data || ! data_length)
        return SBV_ERROR;

    data_pkt        = (sbv_ota_data_pkt_t *)data;
    if (data_pkt->sof != SBV_OTA_SOF)
        return SBV_ERROR;

    if (data_pkt->packet_type != SBV_OTA_PACKET_TYPE_DATA)
        return SBV_ERROR;

    pkt_crc         = data_pkt->crc;
    data_pkt->crc   = 0;
    new_crc         = sbv_ota_msg_crc_calculate((uint8_t *)data_pkt, data_length);
    if(pkt_crc != new_crc)
    {
        /* LOG */
        return SBV_ERROR;
    }

    sbv_rtos_mutex_lock(sbv_ota_msg_rx_instance.mutex);

    rcv_size = data_length - sizeof(sbv_ota_data_pkt_t);
    ret = sbv_cqbuff_write (sbv_ota_msg_rx_instance.rx_queue, data_pkt->data, rcv_size);
    if (ret <= 0)
    {
        /* LOG */
        sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);
        return SBV_ERROR;
    }
    sbv_ota_msg_rx_instance.rcv_data_size += rcv_size;
    if (sbv_ota_msg_rx_instance.rcv_data_size >= sbv_ota_msg_rx_instance.data_size)
    {
        sbv_ota_msg_rx_instance.rcv_data_size   = 0;
        sbv_ota_msg_rx_instance.data_size       = 0;
        sbv_ota_msg_rx_instance.rx_state        = SBV_OTA_STATE_END;
        goto TRIGGER;
    }

    if(sbv_cqbuff_get_size (sbv_ota_msg_rx_instance.rx_queue) < SBV_OTA_PAGES_SIZE)
    {
        sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);
        return SBV_BUSY;
    }

TRIGGER:
    sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);
    /* Trigger event to sbv_ota_update_fw_thread */
    sbv_rtos_event_group_set_bits(sbv_ota_event_group, SBV_OTA_RCV_PAGE);

    return SBV_OK;
}

int
sbv_ota_msg_rx_handle(uint8_t *data, const uint16_t data_length)
{
    int ret = SBV_OK;

    if(! data || ! data_length)
        return SBV_ERROR;

    switch (sbv_ota_msg_rx_instance.rx_state)
    {
    // Intend to allow the code to move through the IDLE state right
    // to the START state when the received OTA msg from peer
    case SBV_OTA_STATE_IDLE:
        sbv_ota_msg_rx_instance.rx_state = SBV_OTA_STATE_START;
    case SBV_OTA_STATE_START:
        ret = sbv_ota_msg_rx_handle_cmd(data, data_length);
        break;

    case SBV_OTA_STATE_HEADER:
        ret = sbv_ota_msg_rx_handle_header(data, data_length);
        break;

    case SBV_OTA_STATE_DATA:
        ret = sbv_ota_msg_handle_data(data, data_length);
        break;

    case SBV_OTA_STATE_END:
        ret = sbv_ota_msg_rx_handle_cmd(data, data_length);
        break;

    default:
        goto EXIT;
    }

    if (ret == SBV_BUSY)
        return SBV_OK;

EXIT:
    sbv_ota_msg_send_resp((ret == SBV_OK) ? SBV_OTA_ACK : SBV_OTA_NACK);
    return ret;
}

int
sbv_ota_msg_handle_resp(uint8_t *data, uint32_t data_length)
{
    sbv_ota_resp_pkt_t *resp_pkt;
    uint32_t pkt_crc, new_crc;

    if(! data || ! data_length)
        return SBV_ERROR;

    if(data_length != sizeof(sbv_ota_resp_pkt_t))
        return SBV_ERROR;

    resp_pkt = (sbv_ota_resp_pkt_t *)data;

    if((resp_pkt->sof != SBV_OTA_SOF) || (resp_pkt->eof != SBV_OTA_EOF))
        return SBV_ERROR;

    if(resp_pkt->packet_type != SBV_OTA_PACKET_TYPE_RESPONSE)
        return SBV_ERROR;

    pkt_crc         = resp_pkt->crc;
    resp_pkt->crc   = 0;
    new_crc         = sbv_ota_msg_crc_calculate((uint8_t *)resp_pkt, data_length);
    if(pkt_crc != new_crc)
    {
        /* LOG */
        return SBV_ERROR;
    }

    sbv_ota_msg_tx_instance.is_ack = (resp_pkt->status == SBV_OTA_ACK) ? SBV_TRUE : SBV_FALSE;
    return SBV_OK;
}

int
sbv_ota_msg_resp_handle(uint8_t *data, const uint16_t data_length)
{
    int ret = SBV_OK;
    sbv_ota_resp_pkt_t *resp_pkt;

    if(! data || ! data_length)
        return SBV_ERROR;

    resp_pkt = (sbv_ota_resp_pkt_t *)data;
    if((resp_pkt->packet_type == SBV_OTA_PACKET_TYPE_RESPONSE))
    {
        ret = sbv_ota_msg_handle_resp(data, data_length);
        if (ret != SBV_OK)
        {
            return ret;
        }
    }

    return SBV_OK;
}