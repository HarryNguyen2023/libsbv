
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

/* State of the OTA process FSM */
typedef enum sbv_ota_state_t
{
	SBV_OTA_STATE_IDLE,
	SBV_OTA_STATE_START,
	SBV_OTA_STATE_HEADER,
	SBV_OTA_STATE_DATA,
	SBV_OTA_STATE_END,
  SBV_OTA_STATE_MAX,
} sbv_ota_state_t;

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
} sbv_ota_cmd_t;

typedef enum sbv_ota_upd_status
{
	SBV_OTA_UPD_SUCCESS,
  SBV_OTA_UDP_FAILED,
} sbv_ota_upd_status;

/* OTA meta data info */
typedef struct sbv_ota_data_info_t
{
	uint32_t packet_size;
	uint32_t packet_crc;
} sbv_ota_data_info_t;

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
 *   1B      1B       8B     4B    1B
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

typedef struct sbv_ota_msg_rx_instance_t
{
  sbv_cqbuff*       rx_queue;
  sbv_cqbuff*       data_queue;
  sbv_rtos_mutex_t  mutex;
  sbv_ota_state_t   rx_state;
  uint32_t          image_size;
  uint32_t          rcvd_image_size;
  uint32_t          current_flash_page_addr;
  uint8_t           is_update_enable;
  uint8_t           is_updating;
} sbv_ota_msg_rx_instance_t;

typedef struct sbv_ota_msg_tx_instance_t
{
  sbv_ota_state_t tx_state;
  sbv_ota_state_t next_state;
  uint8_t         is_updating;
  uint8_t         max_retry;
  uint8_t         is_ack;
} sbv_ota_msg_tx_instance_t;

typedef struct sbv_ota_msg_hw_cb_t
{
  int (*sbv_ota_msg_send) (uint8_t, uint8_t *, uint16_t, uint16_t);
  int (*sbv_ota_reg_cb) (int (*rx_cb)(uint8_t *, const uint16_t));
  uint8_t* (*sbv_ota_rcv_data) (uint16_t*, uint16_t);
} sbv_ota_msg_hw_cb_t;

uint32_t
sbv_ota_msg_crc_calculate (uint8_t *buffer, uint32_t buffer_length);
int
sbv_ota_msg_rx_handle(uint8_t *data, const uint16_t data_length);
void
sbv_ota_handle_state (sbv_ota_state_t current_state, sbv_ota_state_t next_state, void *data);
int
sbv_ota_msg_resp_handle(uint8_t *data, const uint16_t data_length);

#endif /*SBV_OTA_MSG_H*/