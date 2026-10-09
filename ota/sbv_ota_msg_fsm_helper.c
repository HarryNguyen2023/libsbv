#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_log.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_ota_common.h"
#include "sbv_ota_msg.h"
#include "sbv_ota_fsm_common.h"
#include "sbv_ota_msg_fsm_helper.h"

int
sbv_ota_msg_fsm_handle_helper(sbv_cqbuff *pkt_rcv_queue, void *input_param,
                              uint16_t* peer_seq_num, uint8_t packet_type,
                              uint32_t timeout_ms, void *output_data);

int
sbv_ota_msg_fsm_handle_cmd_body(sbv_ota_pkt_common_header_t *pkt_header,
                                sbv_cqbuff *pkt_rcv_queue, uint16_t* peer_seq_num,
                                sbv_ota_cmd_t *cmd_type, uint32_t timeout_ms)
{
    int ret;
    sbv_ota_cmd_pkt_t cmd_pkt;

    memset(&cmd_pkt, 0, sizeof(sbv_ota_cmd_pkt_t));

    if (! pkt_header || ! pkt_rcv_queue || ! peer_seq_num) {
        LOG_ERROR ("Invalid input, nil parameters");
        return SBV_ERROR;
    }

    memcpy (&(cmd_pkt.h), pkt_header, sizeof(sbv_ota_pkt_common_header_t));

    ret = sbv_ota_msg_get_rcv_data (NULL, pkt_rcv_queue, &(cmd_pkt.cmd),
                                    cmd_pkt.h.length, timeout_ms);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to rcv packet content, aborting handle cmd packet process");
        return ret;
    }

    ret = sbv_ota_msg_rx_cmd_packet_validate (&cmd_pkt, *cmd_type, peer_seq_num);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to validate packet content, aborting handle cmd packet process");
        return ret;
    }

    return SBV_OK;
}


int
sbv_ota_msg_fsm_handle_cmd(sbv_cqbuff *pkt_rcv_queue, uint16_t* peer_seq_num,
                           sbv_ota_cmd_t cmd_type, uint32_t timeout_ms)
{
    int ret;
    
    ret = sbv_ota_msg_fsm_handle_helper (pkt_rcv_queue, &cmd_type,
                                        peer_seq_num, SBV_OTA_PACKET_TYPE_CMD,
                                        timeout_ms, NULL);
    return ret;
}

int
sbv_ota_msg_fsm_handle_header_body (sbv_ota_pkt_common_header_t *pkt_header,
                                    sbv_cqbuff *pkt_rcv_queue, uint16_t* peer_seq_num,
                                    uint32_t timeout_ms, sbv_ota_fw_metadata_t *out_fw_metadata)
{
    int ret;
    sbv_ota_header_pkt_t header_pkt;

    memset(&header_pkt, 0, sizeof(sbv_ota_header_pkt_t));

    if (! pkt_header || ! pkt_rcv_queue || ! peer_seq_num || ! out_fw_metadata) {
        LOG_ERROR ("Invalid input, nil parameters");
        return SBV_ERROR;
    }

    memcpy(&(header_pkt.h), pkt_header, sizeof(sbv_ota_header_pkt_t));

    ret = sbv_ota_msg_get_rcv_data (NULL, pkt_rcv_queue, &(header_pkt.data_info),
                                    header_pkt.h.length, timeout_ms);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to rcv packet content, aborting handle header packet process");
        return ret;
    }

    ret = sbv_ota_msg_rx_header_packet_validate (&header_pkt, peer_seq_num);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to validate packet content, aborting handle header packet process");
        return ret;
    }

    memcpy (out_fw_metadata, &header_pkt.data_info, sizeof(sbv_ota_fw_metadata_t));

    return SBV_OK;
}

int
sbv_ota_msg_fsm_handle_header(sbv_cqbuff *pkt_rcv_queue,
                              uint16_t* peer_seq_num, uint32_t timeout_ms,
                              sbv_ota_fw_metadata_t *out_fw_metadata)
{
    int ret;
    
    ret = sbv_ota_msg_fsm_handle_helper(pkt_rcv_queue, NULL,
                                        peer_seq_num, SBV_OTA_PACKET_TYPE_HEADER,
                                        timeout_ms, out_fw_metadata);
    return ret;
}

int
sbv_ota_msg_fsm_handle_data_body(sbv_ota_pkt_common_header_t *pkt_header,
                                 sbv_cqbuff *pkt_rcv_queue, uint16_t* peer_seq_num,
                                 uint32_t timeout_ms, uint32_t rcv_image_size,
                                 uint8_t *out_fw_image)
{
    int ret;
    sbv_ota_data_pkt_t *data_pkt;
    uint32_t total_pkt_len;

    if (! pkt_header || ! pkt_rcv_queue || ! peer_seq_num || ! out_fw_image) {
        LOG_ERROR ("Invalid input, nil parameters");
        return SBV_ERROR;
    }

    total_pkt_len = pkt_header->length + sizeof(sbv_ota_data_pkt_t);
    data_pkt = sbv_rtos_malloc(total_pkt_len);
    if (! data_pkt) {
        LOG_ERROR ("Failed to allocate memory for SBV OTA data packet");
        return SBV_ERROR;
    }

    memcpy (&(data_pkt->h), pkt_header, sizeof (sbv_ota_pkt_common_header_t));

    ret = sbv_ota_msg_get_rcv_data (NULL, pkt_rcv_queue, (data_pkt->data),
                                    data_pkt->h.length, timeout_ms);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to rcv packet content, aborting handle data packet process");
        goto EXIT;
    }

    ret = sbv_ota_msg_rx_data_packet_validate (data_pkt, total_pkt_len, peer_seq_num);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to validate data content, aborting handle cmd packet process");
        goto EXIT;
    }

    if (pkt_header->length > rcv_image_size) {
        LOG_ERROR ("Invalid image size, expected %u, recv %u", rcv_image_size, pkt_header->length);
        ret = SBV_ERROR;
        goto EXIT;
    }

    memcpy (out_fw_image, data_pkt->data, pkt_header->length);

EXIT:
    sbv_rtos_free (data_pkt);
    return ret;
}

int
sbv_ota_msg_fsm_handle_data(sbv_cqbuff *pkt_rcv_queue, uint16_t* peer_seq_num,
                            uint32_t timeout_ms, uint8_t *out_fw_image,
                            uint32_t* total_rcv_image_length,
                            uint32_t expected_total_img_length)
{
    int ret;
    uint32_t rcv_image_size, rcv_image_left_size;

    rcv_image_left_size = (expected_total_img_length - *total_rcv_image_length);
    rcv_image_size = (rcv_image_left_size < SBV_OTA_DATA_MAX_SIZE) ? \
                        rcv_image_left_size : SBV_OTA_DATA_MAX_SIZE;

    ret = sbv_ota_msg_fsm_handle_helper (pkt_rcv_queue, &rcv_image_size,
                                        peer_seq_num, SBV_OTA_PACKET_TYPE_DATA,
                                        timeout_ms, out_fw_image);
    if (ret == SBV_OK) {
        *total_rcv_image_length += rcv_image_size;
        if (*total_rcv_image_length < expected_total_img_length) {
            return SBV_BUSY;
        } else {
            LOG_INFO ("Slave OTA received all fw images from Master OTA");
            return SBV_OK;
        }
    }
    return ret;
}

/*
 * Only allow to receive the packet type that right previous to
 * the current expected packet type due to the risk of master OTA
 * failed to received our response and retry to send the last message
 */
int
sbv_ota_validate_mismatch_packet_type (sbv_ota_pkt_common_header_t* pkt_header,
                                       uint8_t expected_packet_type) {
    if (! pkt_header) {
        LOG_ERROR ("Nil packet header intput");
        return SBV_ERROR;
    }

    switch (pkt_header->packet_type)
    {
    case SBV_OTA_PACKET_TYPE_CMD:
        return (expected_packet_type == SBV_OTA_PACKET_TYPE_HEADER) ? SBV_OK : SBV_ERROR;
    case SBV_OTA_PACKET_TYPE_HEADER:
        return (expected_packet_type == SBV_OTA_PACKET_TYPE_DATA) ? SBV_OK : SBV_ERROR;
    case SBV_OTA_PACKET_TYPE_DATA:
        return (expected_packet_type == SBV_OTA_PACKET_TYPE_CMD) ? SBV_OK : SBV_ERROR;
    default:
        return SBV_ERROR;
    }

    return SBV_ERROR;
}

int
sbv_ota_msg_fsm_handle_helper(sbv_cqbuff *pkt_rcv_queue, void *input_param,
                              uint16_t* peer_seq_num, uint8_t packet_type,
                              uint32_t timeout_ms, void *output_data) {
    int ret = SBV_OK, packet_ret = SBV_OK;
    sbv_ota_pkt_common_header_t common_header;
    sbv_ota_cmd_t *cmd_type;
    sbv_ota_fw_metadata_t fw_metadata;
    uint32_t *rcv_image_length;

    memset(&common_header, 0, sizeof(sbv_ota_pkt_common_header_t));

    if (! pkt_rcv_queue) {
        LOG_ERROR ("Invalid input, nil packet header or nil packet rcv queue");
        return SBV_ERROR;
    }

    ret = sbv_ota_msg_get_rcv_data (NULL, pkt_rcv_queue, &common_header,
                                    sizeof(sbv_ota_pkt_common_header_t), timeout_ms);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to rcv SBV OTA packet header, aborting handle header packet process");
        return ret;
    }

    ret = sbv_ota_packet_header_validate (&common_header, packet_type);
    if (ret != SBV_OK && ret != SVB_OTA_PKT_TYPE_MISMATCH) {
        LOG_ERROR ("Failed to validate SBV OTA packet header, aborting handle header packet process");
        return ret;
    } else if (ret == SVB_OTA_PKT_TYPE_MISMATCH) {
        if (sbv_ota_validate_mismatch_packet_type (&common_header, packet_type) != SBV_OK) {
            LOG_ERROR ("Mismatch in state between master and slave OTA, expected %s, recv %s",
                       sbv_ota_msg_type_to_str(packet_type), sbv_ota_msg_type_to_str(common_header.packet_type));
            return SBV_ERROR;
        }
    }

    switch (common_header.packet_type)
    {
    case SBV_OTA_PACKET_TYPE_CMD:
        cmd_type = (sbv_ota_cmd_t *)input_param;
        if (*cmd_type == SBV_OTA_CMD_START && ret == SVB_OTA_PKT_TYPE_MISMATCH) {
            LOG_ERROR ("Failed to receive the cmd start, mistmatch state between master and slave OTA");
            return SBV_ERROR;
        }
        return sbv_ota_msg_fsm_handle_cmd_body (&common_header, pkt_rcv_queue,
                                                peer_seq_num, cmd_type, timeout_ms);
    case SBV_OTA_PACKET_TYPE_HEADER:
        packet_ret = sbv_ota_msg_fsm_handle_header_body(&common_header, pkt_rcv_queue,
                                                        peer_seq_num, timeout_ms, &fw_metadata);
        if (ret == SBV_OK && packet_ret == SBV_OK && output_data) {
            memcpy (output_data, &fw_metadata, sizeof (sbv_ota_fw_metadata_t));
        }
        return packet_ret;
    case SBV_OTA_PACKET_TYPE_DATA:
        rcv_image_length = (uint32_t *)input_param;
        packet_ret = sbv_ota_msg_fsm_handle_data_body (&common_header, pkt_rcv_queue,
                                                       peer_seq_num, timeout_ms,
                                                       rcv_image_length, output_data);
        if (ret == SBV_OK && packet_ret == SBV_OK && rcv_image_length) {
            rcv_image_length = common_header.length;
        }
        return packet_ret;
    default:
        break;
    }

    return SBV_OK;
}