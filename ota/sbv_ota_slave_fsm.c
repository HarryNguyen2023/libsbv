#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_ota_common.h"
#include "sbv_ota_msg.h"
#include "sbv_ota_fsm_common.h"
#include "sbv_ota_slave_fsm.h"


#define SBV_OTA_SLAVE_AND_INSTALLER_QUEUE_LEN    1
#define SBV_OTA_SLAVE_FSM_PRIORITY               2

#define SBV_OTA_SLAVE_RCV_BUFFER_SIZE   (SBV_OTA_PACKET_MAX_SIZE)
#define SBV_OTA_SLAVE_MSG_TIMEOUT_MS    100
#define SBV_OTA_BLOCKING_MAX_DELAY_MS   (UINT32_MAX)

#define SBV_OTA_SLAVE_SYSTEM_MSG_TIMEOUT_MS 100

#define SBV_OTA_SLAVE_NEXT_STATE(NS,RET,RESP)   \
    (RET!=SBV_OK) ? SBV_OTA_STATE_IDLE :        \
        (((RESP)!=SBV_OTA_ACK) ? SBV_OTA_STATE_IDLE : NS)

void sbv_ota_slave_fsm_idle (sbv_ota_state_t current_state, void *data);
void sbv_ota_slave_fsm_start (sbv_ota_state_t current_state, void *data);
void sbv_ota_slave_fsm_header (sbv_ota_state_t current_state, void *data);
void sbv_ota_slave_fsm_data (sbv_ota_state_t current_state, void *data);
void sbv_ota_slave_fsm_end (sbv_ota_state_t current_state, void *data);
void sbv_ota_slave_fsm_handle_state (void *data);

int sbv_ota_slave_fsm_start_fw_update (void);
int sbv_ota_slave_fsm_send_fw_metadata_to_installelr (void);
int sbv_ota_slave_fsm_send_img_to_installer (void);
int sbv_ota_slave_fsm_stop_fw_update (void);

void sbv_task_ota_update_fw_slave (void* param);

static sbv_rtos_stack_type_t sbv_ota_slave_fsm_stack[STACK_SIZE_BASE * 4];
sbv_rtos_task_handle_t       sbv_ota_slave_handle;

sbv_ota_msg_slave_handler_t sbv_ota_msg_slave_handler;

extern sbv_rtos_queue_handle_t sbv_ota_installer_rx_queue;
sbv_rtos_queue_handle_t sbv_ota_slave_rx_installer_tx_queue;
static uint8_t rcv_buffer[SBV_OTA_SLAVE_RCV_BUFFER_SIZE];

struct sbv_ota_fsm_cb_t sbv_ota_slave_fsm_state[SBV_OTA_STATE_MAX][SBV_OTA_STATE_MAX] = {
    {{SBV_OTA_STATE_IDLE,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_slave_fsm_start},
     {SBV_OTA_STATE_HEADER, sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_END,    sbv_ota_slave_fsm_idle}},
    
    {{SBV_OTA_STATE_IDLE,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_slave_fsm_header},
     {SBV_OTA_STATE_DATA,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_END,    sbv_ota_slave_fsm_idle}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_slave_fsm_data},
     {SBV_OTA_STATE_END,    sbv_ota_slave_fsm_idle}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_END,    sbv_ota_slave_fsm_end}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_slave_fsm_idle},
     {SBV_OTA_STATE_END,    sbv_ota_slave_fsm_idle}},
};

typedef struct sbv_ota_slave_fw_installer_task_t {
    int (*function_task) (void);
} sbv_ota_slave_fw_installer_task_t;

sbv_ota_slave_fw_installer_task_t sbv_ota_slave_fw_installer_tasks[4] = {
    sbv_ota_slave_fsm_start_fw_update,
    sbv_ota_slave_fsm_send_fw_metadata_to_installelr,
    sbv_ota_slave_fsm_send_img_to_installer,
    sbv_ota_slave_fsm_stop_fw_update
};

int
sbv_ota_slave_fsm_fw_installer_tasks_process (void) {
    int ret;

    for (uint8_t i = 0; i < 4; ++i) {
        ret = (sbv_ota_slave_fw_installer_tasks[i].function_task)();
        if (ret != SBV_OK) {
            // LOG
            return ret;
        }
    }

    sbv_ota_msg_slave_handler.is_update_enable = SBV_FALSE;

    return SBV_OK;
}

void
sbv_ota_slave_fsm_init (void)
{
    memset(&sbv_ota_msg_slave_handler, 0, sizeof (sbv_ota_msg_slave_handler_t));

    sbv_ota_msg_slave_handler.state             = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_slave_handler.next_state        = SBV_OTA_STATE_START;
    sbv_ota_msg_slave_handler.is_updating       = SBV_FALSE;
    sbv_ota_msg_slave_handler.is_update_enable  = SBV_TRUE;

    sbv_ota_msg_slave_handler.data_queue = sbv_cqbuff_create (SBV_OTA_SLAVE_RCV_BUFFER_SIZE, 1);

    sbv_ota_slave_rx_installer_tx_queue = sbv_rtos_create_queue(SBV_OTA_SLAVE_AND_INSTALLER_QUEUE_LEN,
                                                                sizeof (sbv_ota_system_msg_t));
    sbv_ota_msg_slave_handler.slave_rx_installer_tx_queue   = sbv_ota_slave_rx_installer_tx_queue;
    sbv_ota_msg_slave_handler.slave_tx_installer_rx_queue   = sbv_ota_installer_rx_queue;

    sbv_rtos_mutex_create (sbv_ota_msg_slave_handler.mu);

    sbv_rtos_task_create(sbv_task_ota_update_fw_slave, "ota_slave", STACK_SIZE_BASE * 4,
                         NULL, SBV_OTA_SLAVE_FSM_PRIORITY, sbv_ota_slave_fsm_stack, &sbv_ota_slave_handle);
}

void
sbv_ota_slave_fsm_reset (void)
{
    sbv_ota_msg_slave_handler.state             = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_slave_handler.next_state        = SBV_OTA_STATE_START;
    sbv_ota_msg_slave_handler.is_updating       = SBV_FALSE;
    sbv_ota_msg_slave_handler.is_update_enable  = SBV_TRUE;

    sbv_cqbuff_flush (sbv_ota_msg_slave_handler.data_queue);
}

static uint8_t
sbv_ota_slave_fsm_is_updating (void) {
    return sbv_ota_msg_slave_handler.is_updating;
}

static uint8_t
sbv_ota_slave_fsm_is_update_enable (void) {
    return sbv_ota_msg_slave_handler.is_update_enable;
}

uint8_t
sbv_ota_slave_fsm_is_updating_locked (void) {
    uint8_t is_updating;

    sbv_rtos_mutex_lock (sbv_ota_msg_slave_handler.mu);

    is_updating = sbv_ota_slave_fsm_is_updating();

    sbv_rtos_mutex_unlock (sbv_ota_msg_slave_handler.mu);
    return is_updating;
}

static sbv_ota_state_t
sbv_ota_slave_fsm_get_current_state (void) {
    return sbv_ota_msg_slave_handler.state;
}

static sbv_ota_state_t
sbv_ota_slave_fsm_get_next_state (void) {
    return sbv_ota_msg_slave_handler.next_state;
}

void
sbv_task_ota_update_fw_slave (void* param)
{
    // LOG
    for(;;)
    {
        sbv_ota_slave_fsm_handle_state (NULL);
    }
}

void sbv_ota_slave_fsm_handle_state (void *data)
{
    sbv_ota_state_t current_state, next_state;

    sbv_rtos_mutex_lock (sbv_ota_msg_slave_handler.mu);

    current_state = sbv_ota_slave_fsm_get_current_state();
    next_state    = sbv_ota_slave_fsm_get_next_state();

    if (! sbv_ota_slave_fsm_is_update_enable ()) {
        // LOG
        sbv_rtos_mutex_unlock (sbv_ota_msg_slave_handler.mu);

        sbv_rtos_task_delay (SBV_RTOS_MAX_DELAY);
        return;
    }

    if (next_state == SBV_OTA_STATE_IDLE) {
        // LOG
        sbv_ota_slave_fsm_reset ();
        sbv_rtos_mutex_unlock (sbv_ota_msg_slave_handler.mu);
        return;
    }

    sbv_ota_fsm_handle_state (sbv_ota_slave_fsm_state,
                              current_state, next_state, data);
    sbv_ota_msg_slave_handler.state = next_state;

    sbv_rtos_mutex_unlock (sbv_ota_msg_slave_handler.mu);
}

int
sbv_ota_slave_fsm_handle_cmd(void *param, sbv_ota_cmd_t cmd_type, uint32_t timeout_ms)
{
    int ret;
    sbv_ota_cmd_pkt_t cmd_pkt;
    sbv_ota_msg_slave_handler_t *slave_handler;

    memset(&cmd_pkt, 0, sizeof(sbv_ota_cmd_pkt_t));

    slave_handler = (sbv_ota_msg_slave_handler_t *)param;
    if (! slave_handler) {
        // LOG
        return -1;
    }

    ret = sbv_ota_msg_get_rcv_data (NULL, slave_handler->data_queue, &cmd_pkt,
                                    rcv_buffer, SBV_OTA_SLAVE_RCV_BUFFER_SIZE,
                                    sizeof(sbv_ota_cmd_pkt_t), timeout_ms);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_msg_rx_cmd_packet_validate (&cmd_pkt, cmd_type, &(slave_handler->peer_seq_num));
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return SBV_OK;
}

int
sbv_ota_slave_fsm_handle_header(void *param, uint32_t timeout_ms)
{
    int ret;
    sbv_ota_header_pkt_t header_pkt;
    sbv_ota_msg_slave_handler_t *slave_handler;

    memset(&header_pkt, 0, sizeof(sbv_ota_header_pkt_t));

    slave_handler = (sbv_ota_msg_slave_handler_t *)param;
    if (! slave_handler) {
        // LOG
        return -1;
    }

    ret = sbv_ota_msg_get_rcv_data (NULL, slave_handler->data_queue, &header_pkt,
                                    rcv_buffer, SBV_OTA_SLAVE_RCV_BUFFER_SIZE,
                                    sizeof(sbv_ota_header_pkt_t), timeout_ms);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_msg_rx_header_packet_validate (&header_pkt, &(slave_handler->peer_seq_num));
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    memcpy (&slave_handler->new_fw_metadata, &header_pkt.data_info, sizeof (sbv_ota_fw_metadata_t));
    
    return SBV_OK;
}

/*
 * This function handles the data (image) packet send by our peer,
 * which break the raw images into chunks and send it chunk-by-chunk to us.
 * Thus, we must handle 2 case
 *      - Full size chunk images (max chunk size)
 *      - The last chunk with perhaps smaller size
 */
int
sbv_ota_slave_fsm_handle_data(void *param, uint32_t timeout_ms)
{
    int ret;
    uint32_t rcv_size, rcv_data_size;
    sbv_ota_data_pkt_t *data_pkt;
    sbv_ota_msg_slave_handler_t *slave_handler;

    slave_handler = (sbv_ota_msg_slave_handler_t *)param;
    if (! slave_handler) {
        // LOG
        return -1;
    }

    rcv_data_size   = (slave_handler->new_fw_metadata.fw_size - slave_handler->current_rcv_image_size);
    rcv_data_size   = (rcv_data_size > SBV_OTA_DATA_MAX_SIZE) ? SBV_OTA_DATA_MAX_SIZE : rcv_data_size;
    rcv_size        = rcv_data_size + sizeof(sbv_ota_data_pkt_t);

    data_pkt = sbv_rtos_malloc(rcv_size);
    if (! data_pkt) {
        // LOG
        return -1;
    }

    ret = sbv_ota_msg_get_rcv_data (NULL, slave_handler->data_queue, data_pkt,
                                    rcv_buffer, SBV_OTA_SLAVE_RCV_BUFFER_SIZE,
                                    rcv_size, timeout_ms);
    if (ret != SBV_OK) {
        // LOG
        goto ERR_EXIT;
    }

    ret = sbv_ota_msg_rx_data_packet_validate (data_pkt, rcv_size, &(slave_handler->peer_seq_num));
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    memcpy (slave_handler->fw_image + slave_handler->current_rcv_image_size, data_pkt->data, rcv_data_size);

    sbv_rtos_free (data_pkt);

    slave_handler->current_rcv_image_size += rcv_data_size;

    if (slave_handler->current_rcv_image_size >= slave_handler->new_fw_metadata.fw_size) {
        // LOG
        return SBV_OK;
    }

    return SBV_BUSY;

ERR_EXIT:
    sbv_rtos_free (data_pkt);
    return SBV_ERROR;
}


void sbv_ota_slave_fsm_idle (sbv_ota_state_t current_state, void *data)
{
    /* Do nothing */
    return;
}

void sbv_ota_slave_fsm_start (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t resp_type;

    if (current_state != SBV_OTA_STATE_IDLE)
    {
        sbv_ota_msg_slave_handler.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    ret = sbv_ota_slave_fsm_handle_cmd(&sbv_ota_msg_slave_handler, SBV_OTA_CMD_START, SBV_OTA_BLOCKING_MAX_DELAY_MS);
    if (ret != SBV_OK) {
        // LOG
    }

    sbv_ota_msg_slave_handler.is_updating = SBV_TRUE;

    resp_type = (ret == SBV_OK) ? SBV_OTA_ACK : SBV_OTA_NACK;

    sbv_ota_msg_slave_handler.seq_num += sizeof(sbv_ota_resp_pkt_t);

    ret = sbv_ota_msg_send_resp (resp_type, sbv_ota_msg_slave_handler.seq_num, SBV_OTA_SLAVE_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        /* LOG */
    }

    sbv_ota_msg_slave_handler.next_state = SBV_OTA_SLAVE_NEXT_STATE (SBV_OTA_STATE_HEADER, ret, resp_type);

    return;
}

void sbv_ota_slave_fsm_header (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t resp_type;

    if (current_state != SBV_OTA_STATE_START
        || ! sbv_ota_slave_fsm_is_updating())
    {
        sbv_ota_msg_slave_handler.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    ret = sbv_ota_slave_fsm_handle_header (&sbv_ota_msg_slave_handler, SBV_OTA_SLAVE_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        /* LOG */
    }

    resp_type = (ret == SBV_OK) ? SBV_OTA_ACK : SBV_OTA_NACK;

    sbv_ota_msg_slave_handler.seq_num += sizeof(sbv_ota_resp_pkt_t);

    ret = sbv_ota_msg_send_resp (resp_type, sbv_ota_msg_slave_handler.seq_num, SBV_OTA_SLAVE_MSG_TIMEOUT_MS);
    if (ret < 0) {
        /* LOG */
    }

    sbv_ota_msg_slave_handler.next_state = SBV_OTA_SLAVE_NEXT_STATE (SBV_OTA_STATE_DATA, ret, resp_type);

    return;
}

static int
sbv_ota_slave_validate_rcv_fw_image_crc (void)
{
    return (sbv_ota_calculate_crc(sbv_ota_msg_slave_handler.fw_image, sbv_ota_msg_slave_handler.current_rcv_image_size) \
                == sbv_ota_msg_slave_handler.new_fw_metadata.fw_crc);
}

void sbv_ota_slave_fsm_data (sbv_ota_state_t current_state, void *data)
{
    int ret1, ret2;
    uint8_t resp_type;

    if (current_state != SBV_OTA_STATE_HEADER
        || ! sbv_ota_slave_fsm_is_updating())
    {
        sbv_ota_msg_slave_handler.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    ret1 = sbv_ota_slave_fsm_handle_data (&sbv_ota_msg_slave_handler, SBV_OTA_SLAVE_MSG_TIMEOUT_MS);
    if (ret1 != SBV_BUSY && ret1 != SBV_OK) {
        // LOG
    }

    resp_type = (ret1 == SBV_OK || ret1 == SBV_BUSY) ? SBV_OTA_ACK : SBV_OTA_NACK;

    sbv_ota_msg_slave_handler.seq_num += sizeof(sbv_ota_resp_pkt_t);

    ret2 = sbv_ota_msg_send_resp (resp_type, sbv_ota_msg_slave_handler.seq_num, SBV_OTA_SLAVE_MSG_TIMEOUT_MS);
    if (ret2 != SBV_OK)
    {
        /* LOG */
    }

    // Not yet receive all the firmware image, stay at the current DATA state
    if (ret1 == SBV_BUSY) {
        // LOG
        return;
    }

    sbv_ota_msg_slave_handler.next_state = SBV_OTA_SLAVE_NEXT_STATE (SBV_OTA_STATE_END, ret2, resp_type);

    return;
}

void sbv_ota_slave_fsm_end (sbv_ota_state_t current_state, void *data)
{
    int ret;
    sbv_ota_upd_status upd_status;

    if (current_state != SBV_OTA_STATE_DATA
        || ! sbv_ota_slave_fsm_is_updating())
    {
        sbv_ota_msg_slave_handler.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    ret = sbv_ota_slave_fsm_handle_cmd(&sbv_ota_msg_slave_handler, SBV_OTA_CMD_END, SBV_OTA_SLAVE_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        // LOG
        goto SEND_REPORT;
    }

    // Verify the whole image hash CRC
    if (sbv_ota_slave_validate_rcv_fw_image_crc () != SBV_TRUE) {
        // LOG
        ret = SBV_ERROR;
        goto SEND_REPORT;
    }

    ret = sbv_ota_slave_fsm_fw_installer_tasks_process ();
    if (ret != SBV_OK) {
        // LOG
    }

SEND_REPORT:
    upd_status = (ret == SBV_OK) ? SBV_OTA_UPD_SUCCESS : SBV_OTA_UDP_FAILED;

    sbv_ota_msg_slave_handler.seq_num += sizeof(sbv_ota_report_pkt_t);

    ret = sbv_ota_msg_send_report (upd_status, &(sbv_ota_msg_slave_handler.new_fw_metadata),
                                   sbv_ota_msg_slave_handler.seq_num, SBV_OTA_SLAVE_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        // LOG
    }

    sbv_ota_msg_slave_handler.next_state  = SBV_OTA_STATE_IDLE;

    sbv_ota_msg_slave_handler.is_updating = SBV_FALSE;

    return;
}

int
sbv_ota_send_system_msg_start (sbv_rtos_queue_handle_t queue, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_START, NULL, timeout_ms);
}

int
sbv_ota_send_system_msg_metadata (sbv_rtos_queue_handle_t queue, void* img_metadata, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_METADATA, img_metadata, timeout_ms);
}

int
sbv_ota_send_system_msg_image (sbv_rtos_queue_handle_t queue, void* img, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_IMG, img, timeout_ms);
}

int
sbv_ota_send_system_msg_stop (sbv_rtos_queue_handle_t queue, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_STOP, NULL, timeout_ms);
}

int
sbv_ota_rcv_system_msg (sbv_rtos_queue_handle_t queue, sbv_ota_system_msg_t* system_msg, uint16_t timeout_ms) {
    sbv_rtos_base_type_t status;

    if (system_msg == NULL) {
        // LOG
        return -1;
    }

    status = sbv_rtos_queue_rcv (queue, system_msg, sbv_rtos_ms_to_tick (timeout_ms));
    if (status != SBV_RTOS_TRUE) {
        // LOG
        return -1;
    }

    return SBV_OK;
}

int
sbv_ota_slave_fsm_system_msg_handle (void) {
    int ret;
    sbv_ota_system_msg_t system_msg;

    memset (&system_msg, 0, sizeof (sbv_ota_system_msg_t));

    ret = sbv_ota_rcv_system_msg (sbv_ota_msg_slave_handler.slave_rx_installer_tx_queue,
                                  &system_msg, SBV_OTA_SLAVE_SYSTEM_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    switch (system_msg.event)
    {
    case SBV_OTA_EVENT_ACK:
        // LOG
        return SBV_OK;
    case SBV_OTA_EVENT_ABORT:
        // LOG
        return SBV_ERROR;
    default:
        // LOG
        return SBV_ERROR;
    }
}

int
sbv_ota_slave_fsm_start_fw_update (void) {
    int ret;

    ret = sbv_ota_send_system_msg_start (sbv_ota_msg_slave_handler.slave_tx_installer_rx_queue, SBV_OTA_SLAVE_SYSTEM_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_slave_fsm_system_msg_handle ();
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return SBV_OK;
}

int
sbv_ota_slave_fsm_send_fw_metadata_to_installelr (void) {
    int ret;

    ret = sbv_ota_send_system_msg_metadata (sbv_ota_msg_slave_handler.slave_tx_installer_rx_queue,
                                            &(sbv_ota_msg_slave_handler.new_fw_metadata),
                                            SBV_OTA_SLAVE_SYSTEM_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_slave_fsm_system_msg_handle ();
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return SBV_OK;
}

int
sbv_ota_slave_fsm_send_img_to_installer (void) {
    int ret;

    ret = sbv_ota_send_system_msg_image (sbv_ota_msg_slave_handler.slave_tx_installer_rx_queue,
                                         sbv_ota_msg_slave_handler.fw_image,
                                         SBV_OTA_SLAVE_SYSTEM_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_slave_fsm_system_msg_handle ();
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return SBV_OK;
}

int
sbv_ota_slave_fsm_stop_fw_update (void) {
    int ret;

    ret = sbv_ota_send_system_msg_stop (sbv_ota_msg_slave_handler.slave_tx_installer_rx_queue, SBV_OTA_SLAVE_SYSTEM_MSG_TIMEOUT_MS);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_slave_fsm_system_msg_handle ();
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    return SBV_OK;
}

