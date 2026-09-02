
#ifndef SBV_OTA_MSG_H
#define SBV_OTA_MSG_H

#include "sbv.h"

#define SBV_OTA_SOF 	0xAA	/* Start of frame */
#define SBV_OTA_EOF 	0xBB	/* End of frame */
#define SBV_OTA_ACK 	0x00	/* ACK */
#define SBV_OTA_NACK	0x01	/* NACK */

#define SBV_OTA_DATA_MAX_SIZE     (1024)
#define SBV_OTA_DATA_OVERHEAD     (6)
#define SBV_OTA_PACKET_MAX_SIZE   (SBV_OTA_DATA_MAX_SIZE + SBV_OTA_DATA_OVERHEAD)

/* SBV OTA Packet type */
typedef enum sbv_ota_pkt_type_t
{
	SBV_OTA_PACKET_TYPE_CMD,
	SBV_OTA_PACKET_TYPE_DATA,
	SBV_OTA_PACKET_TYPE_HEADER,
	SBV_OTA_PACKET_TYPE_RESPONSE,
  SBV_OTA_PACKET_TYPE_REPORT,
} sbv_ota_pkt_type_t;

/* SBV OTA commands */
typedef enum sbv_ota_cmd_t
{
	SBV_OTA_CMD_START,
	SBV_OTA_CMD_END,
	SBV_OTA_CMD_ABORT,
  SBV_OTA_CMD_MAX,
} sbv_ota_cmd_t;

typedef enum sbv_ota_upd_status
{
	SBV_OTA_UPD_SUCCESS,
  SBV_OTA_UDP_FAILED,
} sbv_ota_upd_status;

/*
 * OTA Command format
 *
 * __________________________________
 * |     | Packet |     |     |     |
 * | SOF |  Type  | CMD | CRC | EOF |
 * |_____|________|_____|_____|_____|
 *   1B      1B     1B     4B    1B
 */
typedef struct sbv_ota_cmd_pkt_t
{
  uint8_t   sof;
  uint8_t   packet_type;
  uint8_t   cmd;
  uint32_t  crc;
  uint8_t   eof;
} __attribute__((packed)) sbv_ota_cmd_pkt_t;

/*
 * OTA Header format
 *
 * _____________________________________
 * |     | Packet | Header |     |     |
 * | SOF |  Type  |  Data  | CRC | EOF |
 * |_____|________|________|_____|_____|
 *   1B      1B       8B     4B    1B
 */
typedef struct sbv_ota_header_pkt_t
{
  uint8_t               sof;
  uint8_t               packet_type;
  sbv_ota_fw_metadata_t data_info;
  uint32_t              crc;
  uint8_t               eof;
} __attribute__((packed)) sbv_ota_header_pkt_t;

/*
 * OTA Data format
 *
 * _______________________________
 * |     | Packet |     |        |
 * | SOF |  Type  | CRC |  Data  |
 * |_____|________|_____|________|
 *   1B      1B     4B    nBytes  
 */
typedef struct sbv_ota_data_pkt_t
{
  uint8_t     sof;
  uint8_t     packet_type;
  uint32_t    crc;
  uint8_t     data[];
} __attribute__((packed)) sbv_ota_data_pkt_t;

/*
 * OTA Response format
 *
 * _____________________________________
 * |     | Packet |        |     |     |
 * | SOF |  Type  | Status | CRC | EOF |
 * |_____|________|________|_____|_____|
 *   1B      1B       1B     4B    1B
 */
typedef struct sbv_ota_resp_pkt_t
{
  uint8_t   sof;
  uint8_t   packet_type;
  uint8_t   status;
  uint32_t  crc;
  uint8_t   eof;
} __attribute__((packed)) sbv_ota_resp_pkt_t;

/*
 * OTA Report format
 *
 * _____________________________________________
 * |     | Packet |        | Header |     |     |
 * | SOF |  Type  | Status |  Data  | CRC | EOF |
 * |_____|________|________|________|_____|_____|
 *   1B      1B       1B       8B     4B    1B
 */
typedef struct sbv_ota_report_pkt_t
{
  uint8_t               sof;
  uint8_t               packet_type;
  sbv_ota_upd_status    status;
  sbv_ota_fw_metadata_t upd_fw_metadata;
  uint32_t              crc;
  uint8_t               eof;
} __attribute__((packed)) sbv_ota_report_pkt_t;

typedef struct sbv_ota_msg_hw_cb_t
{
  int (*sbv_ota_msg_send) (uint8_t, uint8_t *, uint16_t, uint16_t);
  int (*sbv_ota_reg_cb) (int (*rx_cb)(uint8_t *, const uint16_t));
  int (*sbv_ota_rcv_data) (void*, uint8_t[], uint16_t, uint32_t);
} sbv_ota_msg_hw_cb_t;

int
sbv_ota_msg_send_resp (uint8_t resp_type, uint16_t timeout_ms);
int
sbv_ota_msg_send_report (const sbv_ota_upd_status upd_status,
                         const sbv_ota_fw_metadata_t *fw_metadata, uint16_t timeout_ms);
int
sbv_ota_msg_send_cmd (sbv_ota_cmd_t cmd_type, uint16_t timeout_ms);
int
sbv_ota_msg_send_data_header(uint8_t *data, sbv_ota_fw_metadata_t* data_info, uint16_t timeout_ms);
int 
sbv_ota_msg_send_data_frame(uint8_t *data, uint32_t data_length, uint16_t timeout_ms);

int
sbv_ota_msg_rx_resp_packet_validate (sbv_ota_resp_pkt_t* resp_pkt);
int
sbv_ota_msg_rx_report_packet_validate (sbv_ota_report_pkt_t* report_pkt);
int
sbv_ota_msg_rx_data_packet_validate (sbv_ota_data_pkt_t* data_pkt, uint16_t pkt_length);
int
sbv_ota_msg_rx_header_packet_validate (sbv_ota_header_pkt_t* head_pkt);
int
sbv_ota_msg_rx_cmd_packet_validate (sbv_ota_cmd_pkt_t* cmd_pkt);
int
sbv_ota_msg_get_rcv_data (void *queue_instance, sbv_cqbuff *queue, void *packet, uint8_t rcv_buffer[],
                          uint16_t buffer_size, int data_size, uint32_t timeout_ms);

#endif /*SBV_OTA_MSG_H*/