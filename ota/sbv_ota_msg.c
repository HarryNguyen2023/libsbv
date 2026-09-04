#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_uart.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_uart.h"
#include "sbv_can.h"
#include "sbv_ota_common.h"
#include "sbv_ota.h"
#include "sbv_ota_msg.h"

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

int
sbv_ota_rcv_data (void* param, uint8_t data[], uint16_t length, uint32_t timeout_ms)
{
    if (sbv_ota_msg_hw_cb.sbv_ota_rcv_data)
    {
        return (sbv_ota_msg_hw_cb.sbv_ota_rcv_data) (param, data, length, timeout_ms);
    }

    return 0;
}

int
sbv_ota_msg_send_resp (uint8_t resp_type, uint16_t seq_num, uint16_t timeout_ms)
{
    uint32_t pkt_crc;
	sbv_ota_resp_pkt_t resp_pkt;

    if((resp_type != SBV_OTA_ACK) && (resp_type != SBV_OTA_NACK))
        return -1;

    resp_pkt.h.sof 			= SBV_OTA_SOF;
    resp_pkt.h.packet_type 	= SBV_OTA_PACKET_TYPE_RESPONSE;
    resp_pkt.h.length       = SBV_OTA_RESP_PACKET_LEN;
    resp_pkt.h.seq_num      = seq_num;
    resp_pkt.h.crc          = 0;
    resp_pkt.status 		= resp_type;

	pkt_crc = sbv_ota_frame_crc((uint8_t *)&resp_pkt, sizeof(sbv_ota_resp_pkt_t));
    resp_pkt.h.crc = pkt_crc;

    return sbv_ota_msg_send((uint8_t *)&resp_pkt, sizeof(sbv_ota_resp_pkt_t), timeout_ms);
}

int
sbv_ota_msg_send_report (const sbv_ota_upd_status upd_status, uint16_t seq_num,
                         const sbv_ota_fw_metadata_t *fw_metadata, uint16_t timeout_ms)
{
    uint32_t pkt_crc;
	sbv_ota_report_pkt_t report_pkt;

    if (! fw_metadata)
        return SBV_ERROR;

    memset(&report_pkt, 0, sizeof(sbv_ota_report_pkt_t));

    report_pkt.h.sof 			= SBV_OTA_SOF;
    report_pkt.h.packet_type 	= SBV_OTA_PACKET_TYPE_REPORT;
    report_pkt.h.length         = SBV_OTA_REP_PACKET_LEN;
    report_pkt.h.seq_num        = seq_num;
    report_pkt.h.crc            = 0;
    report_pkt.status 		    = upd_status;
    memcpy (&report_pkt.upd_fw_metadata, fw_metadata, sizeof(sbv_ota_fw_metadata_t));

	pkt_crc = sbv_ota_frame_crc((uint8_t *)&report_pkt, sizeof(sbv_ota_report_pkt_t));
    report_pkt.h.crc = pkt_crc;

    return sbv_ota_msg_send((uint8_t *)&report_pkt, sizeof(sbv_ota_report_pkt_t), timeout_ms);
}

int
sbv_ota_msg_send_cmd (sbv_ota_cmd_t cmd_type, uint16_t seq_num, uint16_t timeout_ms)
{
    uint32_t pkt_crc;
	sbv_ota_cmd_pkt_t cmd_pkt;

    cmd_pkt.h.sof 			= SBV_OTA_SOF;
    cmd_pkt.h.packet_type 	= SBV_OTA_PACKET_TYPE_CMD;
    cmd_pkt.h.length        = SBV_OTA_CMD_PACKET_LEN;
    cmd_pkt.h.seq_num       = seq_num;
    cmd_pkt.h.crc           = 0;
    cmd_pkt.cmd 		    = cmd_type;

	pkt_crc = sbv_ota_frame_crc((uint8_t *)&cmd_pkt, sizeof(sbv_ota_cmd_pkt_t));
    cmd_pkt.h.crc = pkt_crc;

    return sbv_ota_msg_send((uint8_t *)&cmd_pkt, sizeof(sbv_ota_cmd_pkt_t), timeout_ms);
}

int
sbv_ota_msg_send_data_header(uint8_t *data, sbv_ota_fw_metadata_t* data_info, uint16_t seq_num, uint16_t timeout_ms)
{
    sbv_ota_header_pkt_t header_pkt;
    uint32_t data_crc, pkt_crc;

    if(! data || ! data_info)
        return -1;

    data_crc = sbv_ota_frame_crc((uint8_t *)data, data_info->fw_size);
    if (data_crc != data_info->fw_crc)
    {
        /* LOG */
        return -1;
    }

    header_pkt.h.sof           = SBV_OTA_SOF;
    header_pkt.h.packet_type   = SBV_OTA_PACKET_TYPE_HEADER;
    header_pkt.h.length        = SBV_OTA_HEADER_PACKET_LEN;
    header_pkt.h.seq_num       = seq_num;
    header_pkt.h.crc           = 0;
    memcpy (&header_pkt.data_info, data_info, sizeof (sbv_ota_fw_metadata_t));

    pkt_crc = sbv_ota_frame_crc((uint8_t *)&header_pkt, sizeof(sbv_ota_header_pkt_t));
    header_pkt.h.crc = pkt_crc;

    return sbv_ota_msg_send((uint8_t *)&header_pkt, sizeof(sbv_ota_header_pkt_t), timeout_ms);
}

int 
sbv_ota_msg_send_data_frame(uint8_t *data, uint32_t data_length, uint16_t seq_num, uint16_t timeout_ms)
{
    sbv_ota_data_pkt_t *data_pkt = NULL;
    uint32_t pkt_length;
    int ret;

    if(! data || ! data_length)
        return -1;


    if(data_length > SBV_OTA_DATA_MAX_SIZE)
    {
        /* LOG */
        return -1;
    }

    pkt_length = sizeof(sbv_ota_data_pkt_t) + data_length;
    data_pkt   = sbv_rtos_malloc(pkt_length);
    if(! data_pkt)
    {
        /* LOG */
        return -1;
    }

    data_pkt->h.sof           = SBV_OTA_SOF;
    data_pkt->h.packet_type   = SBV_OTA_PACKET_TYPE_DATA;
    data_pkt->h.length        = data_length;
    data_pkt->h.seq_num       = seq_num;
    data_pkt->h.crc           = 0;
    memcpy(data_pkt->data, data, data_length);
    data_pkt->h.crc           = sbv_ota_frame_crc((uint8_t *)data_pkt, pkt_length);

    ret = sbv_ota_msg_send((uint8_t *)data_pkt, pkt_length, timeout_ms);
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

    return data_length;

ERR:
    if(data_pkt)
    {
        sbv_rtos_free(data_pkt);
        data_pkt = NULL;
    }
    return -1;
}

int
sbv_ota_packet_header_validate (sbv_ota_pkt_common_header_t *header, uint8_t packet_type) {
    uint16_t pkt_default_length;

    if (! header) {
        // LOG
        return SBV_ERROR;
    }

    if(header->sof != SBV_OTA_SOF)
    {
        /* LOG */
        return SBV_ERROR;
    }

    if(header->packet_type != packet_type)
    {
        /* LOG */
        return SBV_ERROR;
    }

    if (header->length == 0 || header->length > SBV_OTA_DATA_MAX_SIZE) {
        // LOG
        return SBV_ERROR;
    }

    switch (packet_type)
    {
    case SBV_OTA_PACKET_TYPE_CMD:
        pkt_default_length = SBV_OTA_CMD_PACKET_LEN;
        break;
    case SBV_OTA_PACKET_TYPE_HEADER:
        pkt_default_length = SBV_OTA_HEADER_PACKET_LEN;
        break;
    case SBV_OTA_PACKET_TYPE_RESPONSE:
        pkt_default_length = SBV_OTA_RESP_PACKET_LEN;
        break;
    case SBV_OTA_PACKET_TYPE_REPORT:
        pkt_default_length = SBV_OTA_REP_PACKET_LEN;
        break;
    
    default:
        pkt_default_length = 0;
        break;
    }

    return (pkt_default_length && pkt_default_length != header->length) ? SBV_ERROR : SBV_OK;
}

int
sbv_ota_msg_rx_cmd_packet_validate (sbv_ota_cmd_pkt_t* cmd_pkt, sbv_ota_cmd_t cmd_type, uint16_t* seq_num)
{
    int ret;
    uint32_t pkt_crc, new_crc;

    if (! cmd_pkt)
        return -1;

    pkt_crc         = cmd_pkt->h.crc;
    cmd_pkt->h.crc  = 0;
    new_crc         = sbv_ota_frame_crc((uint8_t *)cmd_pkt, sizeof(sbv_ota_cmd_pkt_t));
    if(pkt_crc != new_crc)
    {
        /* LOG */
        goto ERR_EXIT;
    }

    if (cmd_type == SBV_OTA_CMD_START) {
        // LOG
        *seq_num = cmd_pkt->h.seq_num;
    } else {
        ret = sbv_ota_seq_num_validate (seq_num, cmd_pkt->h.seq_num, SBV_OTA_CMD_PACKET_LEN);
        if (ret != SBV_OK) {
            // LOG
            return ret;
        }
    }

    return (cmd_pkt->cmd == cmd_type) ? SBV_OK : SBV_ERROR;

ERR_EXIT:
    return -1;
}

int
sbv_ota_msg_rx_header_packet_validate (sbv_ota_header_pkt_t* head_pkt, uint16_t* seq_num)
{
    int ret;
    uint32_t pkt_crc, new_crc;

    if (! head_pkt)
        return -1;

    pkt_crc         = head_pkt->h.crc;
    head_pkt->h.crc = 0;
    new_crc         = sbv_ota_frame_crc((uint8_t *)head_pkt, sizeof(sbv_ota_header_pkt_t));
    if(pkt_crc != new_crc)
    {
        /* LOG */
        goto ERR_EXIT;
    }

    // TODO: Check fw timestampt vs NOW

    // Check firmware size limit
    if (head_pkt->data_info.fw_size == 0 || head_pkt->data_info.fw_size > SBV_OTA_SLOT_MAX_SIZE) {
        // LOG
        return -1;
    }

    ret = sbv_ota_seq_num_validate (seq_num, head_pkt->h.seq_num, SBV_OTA_HEADER_PACKET_LEN);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return SBV_OK;

ERR_EXIT:
    return -1;
}

int
sbv_ota_msg_rx_data_packet_validate (sbv_ota_data_pkt_t* data_pkt, uint16_t pkt_length, uint16_t* seq_num)
{
    int ret;
    uint32_t pkt_crc, new_crc;

    if (! data_pkt)
        return -1;

    pkt_crc         = data_pkt->h.crc;
    data_pkt->h.crc = 0;
    new_crc         = sbv_ota_frame_crc((uint8_t *)data_pkt, pkt_length);
    if(pkt_crc != new_crc)
    {
        /* LOG */
        goto ERR_EXIT;
    }

    ret = sbv_ota_seq_num_validate (seq_num, data_pkt->h.seq_num, data_pkt->h.length);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return SBV_OK;

ERR_EXIT:
    return -1;
}

int
sbv_ota_msg_rx_resp_packet_validate (sbv_ota_resp_pkt_t* resp_pkt, uint16_t* seq_num)
{
    int ret;
    uint32_t pkt_crc, new_crc;

    if (! resp_pkt)
        return -1;

    pkt_crc         = resp_pkt->h.crc;
    resp_pkt->h.crc = 0;
    new_crc         = sbv_ota_frame_crc((uint8_t *)resp_pkt, sizeof(sbv_ota_resp_pkt_t));
    if(pkt_crc != new_crc)
    {
        /* LOG */
        goto ERR_EXIT;
    }

    ret = sbv_ota_seq_num_validate (seq_num, resp_pkt->h.seq_num, SBV_OTA_RESP_PACKET_LEN);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return 0;

ERR_EXIT:
    return -1;
}

int
sbv_ota_msg_rx_report_packet_validate (sbv_ota_report_pkt_t* report_pkt, uint16_t* seq_num)
{
    int ret;
    uint16_t expected_seq_num;
    uint32_t pkt_crc, new_crc;

    if (! report_pkt)
        return -1;

    pkt_crc            = report_pkt->h.crc;
    report_pkt->h.crc  = 0;
    new_crc            = sbv_ota_frame_crc((uint8_t *)report_pkt, sizeof(sbv_ota_report_pkt_t));
    if(pkt_crc != new_crc)
    {
        /* LOG */
        goto ERR_EXIT;
    }

    ret = sbv_ota_seq_num_validate (seq_num, report_pkt->h.seq_num, SBV_OTA_REP_PACKET_LEN);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return 0;

ERR_EXIT:
    return -1;
}

int
sbv_ota_msg_get_rcv_data (void *queue_instance, sbv_cqbuff *queue, void *packet, uint8_t rcv_buffer[],
                          uint16_t buffer_size, int data_size, uint32_t timeout_ms)
{
    int ret, data_len;
    uint32_t start_tick = sbv_rtos_get_tick();

    if (! queue || ! rcv_buffer || buffer_size == 0 || ! data_size || ! packet) {
        // LOG
        return -1;
    }

    while ((sbv_rtos_get_tick() - start_tick < sbv_rtos_ms_to_tick(timeout_ms))
            && sbv_cqbuff_get_size (queue) < data_size) {
        data_len = sbv_ota_rcv_data (queue_instance, rcv_buffer, buffer_size, timeout_ms);
        if (data_len <= 0) {
            // LOG
            goto ERR_EXIT;
        }

        ret = sbv_cqbuff_write (queue, rcv_buffer, data_len);
        if (ret != data_len) {
            /* LOG */
            goto ERR_EXIT;
        }
    }

    if(sbv_cqbuff_get_size (queue) < data_size) {
       // LOG
        goto ERR_EXIT;
    }

    ret = sbv_cqbuff_read(queue, (unsigned char *)packet, data_size);
    if (ret != data_size) {
        /* LOG */
        goto ERR_EXIT;
    }

    return SBV_OK;

ERR_EXIT:
    sbv_cqbuff_flush (queue);
    return SBV_ERROR;
}