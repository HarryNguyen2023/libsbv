
#ifndef SBV_OTA_MSG_H
#define SBV_OTA_MSG_H

#include "sbv.h"

#define SBV_OTA_SOF 	0xAA	/* Start of frame */
#define SBV_OTA_ACK 	0x00	/* ACK */
#define SBV_OTA_NACK	0x01	/* NACK */

#define SBV_OTA_DATA_MAX_SIZE     (1024)
#define SBV_OTA_DATA_OVERHEAD     (10)
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

typedef struct sbv_ota_pkt_common_header_t {
  uint8_t   sof;
  uint8_t   packet_type;
  uint16_t  length;
#define SBV_OTA_CMD_PACKET_LEN    (1)
#define SBV_OTA_HEADER_PACKET_LEN (sizeof(sbv_ota_fw_metadata_t))
#define SBV_OTA_RESP_PACKET_LEN   (1)
#define SBV_OTA_REP_PACKET_LEN    (sizeof (sbv_ota_report_pkt_t) - sizeof(sbv_ota_pkt_common_header_t))
  uint16_t  seq_num;
  uint32_t  crc;
} __attribute__((packed)) sbv_ota_pkt_common_header_t;

/*
 * OTA Command format
 *
 * ________________________________________
 * |     | Packet |     |     |     |     |
 * | SOF |  Type  | LEN | SEQ | CRC | CMD |
 * |_____|________|_____|_____|_____|_____|
 *   1B      1B     2B    2B     4B    1B
 */
typedef struct sbv_ota_cmd_pkt_t
{
  sbv_ota_pkt_common_header_t h;
  uint8_t                     cmd;
} __attribute__((packed)) sbv_ota_cmd_pkt_t;

/*
 * OTA Header format
 *
 * ___________________________________________
 * |     | Packet |     |     |     | HEADER |
 * | SOF |  Type  | LEN | SEQ | CRC |  DATA  |
 * |_____|________|_____|_____|_____|________|
 *   1B      1B     2B    2B    4B     40B
 */
typedef struct sbv_ota_header_pkt_t
{
  sbv_ota_pkt_common_header_t h;
  sbv_ota_fw_metadata_t       data_info;
} __attribute__((packed)) sbv_ota_header_pkt_t;

/*
 * OTA Data format
 *
 * __________________________________________
 * |     | Packet |     |     |     |        |
 * | SOF |  Type  | LEN | SEQ | CRC |  Data  |
 * |_____|________|_____|_____|_____|________|
 *   1B      1B     2B    2B     4B    nBytes  
 */
typedef struct sbv_ota_data_pkt_t
{
  sbv_ota_pkt_common_header_t h;
  uint8_t                     data[];
} __attribute__((packed)) sbv_ota_data_pkt_t;

/*
 * OTA Response format
 *
 * ___________________________________________
 * |     | Packet |     |     |     |        |
 * | SOF |  Type  | LEN | SEQ | CRC | Status |
 * |_____|________|_____|_____|_____|________|
 *   1B      1B     2B    2B     4B     1B
 */
typedef struct sbv_ota_resp_pkt_t
{
  sbv_ota_pkt_common_header_t h;
  uint8_t                     status;
} __attribute__((packed)) sbv_ota_resp_pkt_t;

/*
 * OTA Report format
 *
 * ____________________________________________________
 * |     | Packet |     |     |     |        | Header |
 * | SOF |  Type  | LEN | SEQ_| CRC | Status |  Data  |
 * |_____|________|_____|_____|_____|________|________|
 *   1B      1B     2B    2B     4B     1B       40B
 */
typedef struct sbv_ota_report_pkt_t
{
  sbv_ota_pkt_common_header_t h;
  sbv_ota_upd_status          status;
  sbv_ota_fw_metadata_t       upd_fw_metadata;
} __attribute__((packed)) sbv_ota_report_pkt_t;

typedef struct sbv_ota_msg_hw_cb_t
{
  int (*sbv_ota_msg_send) (uint8_t, uint8_t *, uint16_t, uint16_t);
  int (*sbv_ota_reg_cb) (int (*rx_cb)(uint8_t *, const uint16_t));
  int (*sbv_ota_rcv_data) (void*, uint8_t[], uint16_t, uint32_t);
} sbv_ota_msg_hw_cb_t;

int
sbv_ota_msg_send_resp (uint8_t resp_type, uint16_t seq_num, uint16_t timeout_ms);
int
sbv_ota_msg_send_report (const sbv_ota_upd_status upd_status, uint16_t seq_num,
                         const sbv_ota_fw_metadata_t *fw_metadata, uint16_t timeout_ms);
int
sbv_ota_msg_send_cmd (sbv_ota_cmd_t cmd_type, uint16_t seq_num, uint16_t timeout_ms);
int
sbv_ota_msg_send_data_header(uint8_t *data, sbv_ota_fw_metadata_t* data_info, uint16_t seq_num, uint16_t timeout_ms);
int 
sbv_ota_msg_send_data_frame(uint8_t *data, uint32_t data_length, uint16_t seq_num, uint16_t timeout_ms);

int
sbv_ota_packet_header_validate (sbv_ota_pkt_common_header_t *header, uint8_t packet_type);
int
sbv_ota_msg_rx_resp_packet_validate (sbv_ota_resp_pkt_t* resp_pkt, uint16_t* seq_num);
int
sbv_ota_msg_rx_report_packet_validate (sbv_ota_report_pkt_t* report_pkt, uint16_t* seq_num);
int
sbv_ota_msg_rx_data_packet_validate (sbv_ota_data_pkt_t* data_pkt, uint16_t pkt_length, uint16_t* seq_num);
int
sbv_ota_msg_rx_header_packet_validate (sbv_ota_header_pkt_t* head_pkt, uint16_t* seq_num);
int
sbv_ota_msg_rx_cmd_packet_validate (sbv_ota_cmd_pkt_t* cmd_pkt, sbv_ota_cmd_t cmd_type, uint16_t* seq_num);
int
sbv_ota_msg_get_rcv_data (void *queue_instance, sbv_cqbuff *queue, void *packet, uint8_t rcv_buffer[],
                          uint16_t buffer_size, int data_size, uint32_t timeout_ms);

#endif /*SBV_OTA_MSG_H*/