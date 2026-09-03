#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_ota_common.h"
#include "sbv_ota_msg.h"
#include "sbv_ota_fsm_common.h"
#include "sbv_ota_master_fsm.h"

#define SBV_OTA_MASTER_TX_MAX_RETRY    5
#define SBV_OTA_MASTER_RX_QUEUE_LEN    5
#define SBV_OTA_MASTER_PRIO            2

#define SBV_OTA_MASTER_RCV_BUFFER_SIZE      (sizeof(sbv_ota_report_pkt_t))
#define SBV_OTA_MASTER_MSG_TIMEOUT_MS       100
#define SBV_OTA_MASTER_END_MSG_TIMEOUT_MS   (10 * 1000)

#define SBV_OTA_MASTER_NEXT_STATE(NS,T,ACK) \
    ((T) == SBV_OTA_MASTER_TX_MAX_RETRY) ? SBV_OTA_STATE_IDLE : \
        (((ACK)!=SBV_TRUE) ? SBV_OTA_STATE_IDLE : NS)

void sbv_ota_master_fsm_idle (sbv_ota_state_t current_state, void *data);
void sbv_ota_master_fsm_start (sbv_ota_state_t current_state, void *data);
void sbv_ota_master_fsm_header (sbv_ota_state_t current_state, void *data);
void sbv_ota_master_fsm_data (sbv_ota_state_t current_state, void *data);
void sbv_ota_master_fsm_end (sbv_ota_state_t current_state, void *data);

void sbv_ota_master_fsm_handle_state (void *data);
void sbv_task_ota_update_fw_master (void* param);
int sbv_ota_master_fsm_handle_resp(void *param, uint32_t timeout_ms);
int sbv_ota_master_fsm_handle_report(void *param, uint32_t timeout_ms);

static sbv_rtos_stack_type_t sbv_ota_master_fsm_stack[STACK_SIZE_BASE * 4];
sbv_rtos_task_handle_t       sbv_ota_master_handle;

sbv_ota_msg_master_handler_t sbv_ota_msg_master_handler;

sbv_rtos_queue_handle_t sbv_ota_master_rx_queue;
static uint8_t rcv_buffer[SBV_OTA_MASTER_RCV_BUFFER_SIZE];

struct sbv_ota_fsm_cb_t sbv_ota_master_fsm_state[SBV_OTA_STATE_MAX][SBV_OTA_STATE_MAX] = {
    {{SBV_OTA_STATE_IDLE,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_master_fsm_start},
     {SBV_OTA_STATE_HEADER, sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_END,    sbv_ota_master_fsm_idle}},
    
    {{SBV_OTA_STATE_IDLE,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_master_fsm_header},
     {SBV_OTA_STATE_DATA,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_END,    sbv_ota_master_fsm_idle}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_master_fsm_data},
     {SBV_OTA_STATE_END,    sbv_ota_master_fsm_idle}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_END,    sbv_ota_master_fsm_end}},

    {{SBV_OTA_STATE_IDLE,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_START,  sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_HEADER, sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_DATA,   sbv_ota_master_fsm_idle},
     {SBV_OTA_STATE_END,    sbv_ota_master_fsm_idle}},
};

void
sbv_ota_master_fsm_init (void)
{
    memset(&sbv_ota_msg_master_handler, 0, sizeof (sbv_ota_msg_master_handler_t));

    sbv_ota_msg_master_handler.max_retry    = SBV_OTA_MASTER_TX_MAX_RETRY;
    sbv_ota_msg_master_handler.state        = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_master_handler.next_state   = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_master_handler.is_updating  = SBV_FALSE;
    sbv_ota_msg_master_handler.is_abort     = SBV_FALSE;

    sbv_ota_msg_master_handler.data_queue   = sbv_cqbuff_create (SBV_OTA_MASTER_RCV_BUFFER_SIZE, 1);

    sbv_ota_master_rx_queue                 = sbv_rtos_create_queue (SBV_OTA_MASTER_RX_QUEUE_LEN, sizeof (sbv_ota_system_msg_t));
    sbv_ota_msg_master_handler.rx_queue     = sbv_ota_master_rx_queue;

    sbv_rtos_mutex_create (sbv_ota_msg_master_handler.mu);

    sbv_rtos_task_create(sbv_task_ota_update_fw_master, "ota_master", STACK_SIZE_BASE * 4,
                         NULL, SBV_OTA_MASTER_PRIO, sbv_ota_master_fsm_stack, &sbv_ota_master_handle);
}

void
sbv_ota_master_fsm_reset (void)
{
    sbv_ota_msg_master_handler.state        = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_master_handler.next_state   = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_master_handler.is_updating  = SBV_FALSE;
    sbv_ota_msg_master_handler.is_abort     = SBV_FALSE;
    sbv_ota_msg_master_handler.is_ack       = SBV_FALSE;

    sbv_ota_msg_master_handler.seq_num      = 0;
    sbv_ota_msg_master_handler.peer_seq_num = 0;

    sbv_cqbuff_flush (sbv_ota_msg_master_handler.data_queue);
}

static uint8_t
sbv_ota_master_fsm_is_updating (void) {
    return sbv_ota_msg_master_handler.is_updating;
}

uint8_t
sbv_ota_master_fsm_is_updating_locked (void) {
    uint8_t is_updating;

    sbv_rtos_mutex_lock (sbv_ota_msg_master_handler.mu);

    is_updating = sbv_ota_master_fsm_is_updating();

    sbv_rtos_mutex_unlock (sbv_ota_msg_master_handler.mu);
    return is_updating;
}

static uint8_t
sbv_ota_master_fsm_is_acknowledged (void) {
    return sbv_ota_msg_master_handler.is_ack;
}

static uint8_t
sbv_ota_master_fsm_is_update_aborted (void) {
    return sbv_ota_msg_master_handler.is_abort;
}

static sbv_ota_state_t
sbv_ota_master_fsm_get_current_state (void) {
    return sbv_ota_msg_master_handler.state;
}

static sbv_ota_state_t
sbv_ota_master_fsm_get_next_state (void) {
    return sbv_ota_msg_master_handler.next_state;
}

void
sbv_task_ota_update_fw_master (void* param)
{
    uint16_t delay_ms = 1000;

    for(;;)
    {
        if (sbv_ota_master_fsm_is_updating_locked())
        {
            sbv_ota_master_fsm_handle_state (NULL);
        }
        else
        {
            // Blocking and waiting for trigger update
            //
            // sbv_ota_msg_master_handler.is_updating = SBV_TRUE;
            // sbv_ota_msg_master_handler.next_state  = SBV_OTA_STATE_START;
            sbv_rtos_task_delay (sbv_rtos_ms_to_tick (delay_ms));
        }
    }
}

void sbv_ota_master_fsm_handle_state (void *data)
{
    sbv_ota_state_t current_state, next_state;

    sbv_rtos_mutex_lock (sbv_ota_msg_master_handler.mu);

    current_state = sbv_ota_master_fsm_get_current_state();
    next_state    = sbv_ota_master_fsm_get_next_state();

    if (next_state == SBV_OTA_STATE_IDLE
        || sbv_ota_master_fsm_is_update_aborted())
    {
        // LOG
        sbv_ota_master_fsm_reset ();

        sbv_rtos_mutex_unlock (sbv_ota_msg_master_handler.mu);
        return;
    }

    sbv_ota_fsm_handle_state (sbv_ota_master_fsm_state,
                              current_state, next_state, data);
    sbv_ota_msg_master_handler.state = next_state;

    sbv_rtos_mutex_unlock (sbv_ota_msg_master_handler.mu);
}

void sbv_ota_master_fsm_idle (sbv_ota_state_t current_state, void *data)
{
    /* Do nothing */
    return;
}

void sbv_ota_master_fsm_start (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t retry_time;

    if (current_state != SBV_OTA_STATE_IDLE)
    {
        sbv_ota_msg_master_handler.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    sbv_ota_msg_master_handler.seq_num = sbv_ota_get_random_seq_number ();

    retry_time = 0;
    while (retry_time < sbv_ota_msg_master_handler.max_retry)
    {
        sbv_ota_msg_master_handler.is_ack = SBV_FALSE;
        ret = sbv_ota_msg_send_cmd (SBV_OTA_CMD_START, sbv_ota_msg_master_handler.seq_num, SBV_OTA_MASTER_MSG_TIMEOUT_MS);
        if (ret < 0)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* This function will call the callback function for handling respose from peer */
        ret = sbv_ota_master_fsm_handle_resp (&sbv_ota_msg_master_handler, SBV_OTA_MASTER_MSG_TIMEOUT_MS);
        if (ret != SBV_OK)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        if (! sbv_ota_master_fsm_is_acknowledged()) {
            retry_time++;
            continue;
        }

        /* LOG */
        break;
    }

    sbv_ota_msg_master_handler.next_state = SBV_OTA_MASTER_NEXT_STATE (SBV_OTA_STATE_HEADER,
                                                                       retry_time,
                                                                       sbv_ota_master_fsm_is_acknowledged());

    return;
}

void sbv_ota_master_fsm_header (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t retry_time, *images;
    sbv_ota_fw_metadata_t data_info;

    if (current_state != SBV_OTA_STATE_START)
    {
        sbv_ota_msg_master_handler.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    // TODO: Read the images data from the filesystem
    //
    //

    sbv_ota_msg_master_handler.seq_num += SBV_OTA_HEADER_PACKET_LEN;

    retry_time = 0;
    while (retry_time < sbv_ota_msg_master_handler.max_retry)
    {
        sbv_ota_msg_master_handler.is_ack = SBV_FALSE;
        ret = sbv_ota_msg_send_data_header (images, &data_info, sbv_ota_msg_master_handler.seq_num, SBV_OTA_MASTER_MSG_TIMEOUT_MS);
        if (ret < 0)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* This function will call the callback function for handling respose from peer */
        ret = sbv_ota_master_fsm_handle_resp (&sbv_ota_msg_master_handler, SBV_OTA_MASTER_MSG_TIMEOUT_MS);
        if (ret != SBV_OK)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        if (! sbv_ota_master_fsm_is_acknowledged()) {
            retry_time++;
            continue;
        }

        /* LOG */
        break;
    }

    sbv_ota_msg_master_handler.next_state = SBV_OTA_MASTER_NEXT_STATE (SBV_OTA_STATE_DATA,
                                                                       retry_time,
                                                                       sbv_ota_master_fsm_is_acknowledged());

    return;
}

void sbv_ota_master_fsm_data (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t retry_time, *images;
    uint16_t image_length, chunk_length;

    if (current_state != SBV_OTA_STATE_HEADER)
    {
        sbv_ota_msg_master_handler.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    // Read the images from the filesystem
    //
    //

    while (image_length > 0)
    {
        chunk_length = (image_length > SBV_OTA_DATA_MAX_SIZE) ? SBV_OTA_DATA_MAX_SIZE : image_length;
        image_length = (image_length > SBV_OTA_DATA_MAX_SIZE) ? (image_length - SBV_OTA_DATA_MAX_SIZE) : 0;

        sbv_ota_msg_master_handler.seq_num += chunk_length;

        retry_time   = 0;
        while (retry_time < sbv_ota_msg_master_handler.max_retry)
        {
            sbv_ota_msg_master_handler.is_ack = SBV_FALSE;
            ret = sbv_ota_msg_send_data_frame (images, chunk_length, sbv_ota_msg_master_handler.seq_num, SBV_OTA_MASTER_MSG_TIMEOUT_MS);
            if (ret < 0)
            {
                /* LOG */
                retry_time++;
                continue;
            }

            /* This function will call the callback function for handling respose from peer */
            ret = sbv_ota_master_fsm_handle_resp (&sbv_ota_msg_master_handler, SBV_OTA_MASTER_MSG_TIMEOUT_MS);
            if (ret != SBV_OK)
            {
                /* LOG */
                retry_time++;
                continue;
            }

            if (! sbv_ota_master_fsm_is_acknowledged()) {
                retry_time++;
                continue;
            }

            /* LOG */
            break;
        }
        images += chunk_length;
    }
    
    

    sbv_ota_msg_master_handler.next_state = SBV_OTA_MASTER_NEXT_STATE(SBV_OTA_STATE_END,
                                                                      retry_time,
                                                                      sbv_ota_master_fsm_is_acknowledged());

    return;
}

void sbv_ota_master_fsm_end (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t retry_time;

    if (current_state != SBV_OTA_STATE_DATA)
    {
        sbv_ota_msg_master_handler.next_state = SBV_OTA_STATE_IDLE;
        return;
    }

    sbv_ota_msg_master_handler.seq_num += SBV_OTA_CMD_PACKET_LEN;

    retry_time = 0;
    while (retry_time < sbv_ota_msg_master_handler.max_retry)
    {
        sbv_ota_msg_master_handler.is_ack = SBV_FALSE;
        ret = sbv_ota_msg_send_cmd (SBV_OTA_CMD_END, sbv_ota_msg_master_handler.seq_num, SBV_OTA_MASTER_MSG_TIMEOUT_MS);
        if (ret < 0)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        /* This function will call the callback function for handling respose from peer */
        ret = sbv_ota_master_fsm_handle_report (&sbv_ota_msg_master_handler, SBV_OTA_MASTER_END_MSG_TIMEOUT_MS);
        if (ret != SBV_OK)
        {
            /* LOG */
            retry_time++;
            continue;
        }

        if (! sbv_ota_master_fsm_is_acknowledged()) {
            retry_time++;
            continue;
        }

        /* LOG */
        break;
    }

    sbv_ota_msg_master_handler.next_state = SBV_OTA_MASTER_NEXT_STATE (SBV_OTA_STATE_IDLE,
                                                                       retry_time,
                                                                       sbv_ota_master_fsm_is_acknowledged());
    sbv_ota_msg_master_handler.is_updating = SBV_FALSE;

    return;
}

int
sbv_ota_master_fsm_handle_resp(void *param, uint32_t timeout_ms)
{
    int ret;
    sbv_ota_resp_pkt_t resp_pkt;
    sbv_ota_msg_master_handler_t *master_handler;

    memset(&resp_pkt, 0, sizeof(sbv_ota_resp_pkt_t));

    master_handler = (sbv_ota_msg_master_handler_t *)param;
    if (! master_handler) {
        // LOG
        return -1;
    }

    ret = sbv_ota_msg_get_rcv_data (NULL, master_handler->data_queue, &(resp_pkt.h),
                                    rcv_buffer, SBV_OTA_MASTER_RCV_BUFFER_SIZE,
                                    sizeof(sbv_ota_pkt_common_header_t), timeout_ms);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_packet_header_validate (&(resp_pkt.h), SBV_OTA_PACKET_TYPE_RESPONSE);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }
    
    ret = sbv_ota_msg_get_rcv_data (NULL, master_handler->data_queue, &(resp_pkt.status),
                                    rcv_buffer, SBV_OTA_MASTER_RCV_BUFFER_SIZE,
                                    resp_pkt.h.length, timeout_ms);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_msg_rx_resp_packet_validate (&resp_pkt, &(master_handler->peer_seq_num));
    if (ret != SBV_OK && ret != SVB_OTA_SEQ_DUP) {
        // LOG
        return ret;
    }

    master_handler->is_ack = (resp_pkt.status == SBV_OTA_ACK) ? SBV_TRUE : SBV_FALSE;
    return SBV_OK;
}

int
sbv_ota_master_fsm_handle_report(void *param, uint32_t timeout_ms)
{
    int ret;
    sbv_ota_report_pkt_t report_pkt;
    sbv_ota_msg_master_handler_t *master_handler;

    memset(&report_pkt, 0, sizeof(sbv_ota_report_pkt_t));

    master_handler = (sbv_ota_msg_master_handler_t *)param;
    if (! master_handler) {
        // LOG
        return -1;
    }

    ret = sbv_ota_msg_get_rcv_data (NULL, master_handler->data_queue, &(report_pkt.h),
                                    rcv_buffer, SBV_OTA_MASTER_RCV_BUFFER_SIZE,
                                    sizeof(sbv_ota_pkt_common_header_t), timeout_ms);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_packet_header_validate (&(report_pkt.h), SBV_OTA_PACKET_TYPE_REPORT);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_msg_get_rcv_data (NULL, master_handler->data_queue, &(report_pkt.status),
                                    rcv_buffer, SBV_OTA_MASTER_RCV_BUFFER_SIZE,
                                    report_pkt.h.length, timeout_ms);
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    ret = sbv_ota_msg_rx_report_packet_validate (&report_pkt, &(master_handler->peer_seq_num));
    if (ret != SBV_OK) {
        // LOG
        return ret;
    }

    // TODO: Save the report of the peer to filesystem
    // Send event over telemetry

    return SBV_OK;
}