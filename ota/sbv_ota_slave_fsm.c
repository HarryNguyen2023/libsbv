#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_log.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_ota_common.h"
#include "sbv_ota_msg.h"
#include "sbv_ota_fsm_common.h"
#include "sbv_ota_slave_fsm.h"
#include "sbv_ota_msg_fsm_helper.h"

#define SBV_OTA_SLAVE_AND_INSTALLER_QUEUE_LEN    1
#define SBV_OTA_SLAVE_FSM_PRIORITY               2

#define SBV_OTA_SLAVE_RCV_BUFFER_SIZE   (SBV_OTA_PACKET_MAX_SIZE)
#define SBV_OTA_SLAVE_MSG_TIMEOUT_MS    100

#define SBV_OTA_SLAVE_MSG_MAX_RETRY     3

#define SBV_OTA_SLAVE_SYSTEM_MSG_TX_TIMEOUT_MS  100
#define SBV_OTA_SLAVE_SYSTEM_MSG_RX_TIMEOUT_MS  (2 * 1000)

#define SBV_OTA_SLAVE_NEXT_STATE(CS,NS,RETURN,RESPONSE)   \
    ((RETURN)!=SBV_OK) ? (((RETURN)==SVB_OTA_SEQ_DUP) ? CS : SBV_OTA_STATE_IDLE) : \
        (((RESPONSE)!=SBV_OTA_ACK) ? SBV_OTA_STATE_IDLE : NS)

int sbv_ota_slave_fsm_idle (sbv_ota_state_t current_state, void *data);
int sbv_ota_slave_fsm_start (sbv_ota_state_t current_state, void *data);
int sbv_ota_slave_fsm_header (sbv_ota_state_t current_state, void *data);
int sbv_ota_slave_fsm_data (sbv_ota_state_t current_state, void *data);
int sbv_ota_slave_fsm_end (sbv_ota_state_t current_state, void *data);
void sbv_ota_slave_fsm_handle_state (void *data);

int sbv_ota_slave_fsm_start_fw_update (void);
int sbv_ota_slave_fsm_send_img_to_installer (void);
int sbv_ota_slave_fsm_stop_fw_update (void);

void sbv_task_ota_update_fw_slave (void* param);

static sbv_rtos_stack_type_t sbv_ota_slave_fsm_stack[STACK_SIZE_BASE * 4];
sbv_rtos_static_task_t       sbv_ota_slave_handle;

sbv_ota_msg_slave_handler_t sbv_ota_msg_slave_handler;

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

typedef int (*sbv_ota_slave_fw_installer_func_t)(void);

sbv_ota_slave_fw_installer_func_t sbv_ota_slave_fw_installer_tasks[3] = {
    sbv_ota_slave_fsm_start_fw_update,
    sbv_ota_slave_fsm_send_img_to_installer,
    sbv_ota_slave_fsm_stop_fw_update
};

int
sbv_ota_slave_fsm_fw_installer_tasks_process (void) {
    int ret;

    for (uint8_t i = 0; i < 3; ++i) {
        ret = (sbv_ota_slave_fw_installer_tasks[i])();
        if (ret != SBV_OK) {
            LOG_ERROR ("Failed to process fw installer task %u, aborting the process", i);
            return ret;
        }
    }

    sbv_ota_msg_slave_handler.is_update_enable = SBV_FALSE;

    return SBV_OK;
}

void
sbv_ota_slave_fsm_init (void* param)
{
    sbv_ota_ipc_t *ipc;

    memset(&sbv_ota_msg_slave_handler, 0, sizeof (sbv_ota_msg_slave_handler_t));

    ipc = (sbv_ota_ipc_t *)param;
    if (! ipc) {
        LOG_ERROR ("Invalid input, OTA IPC queue is nil");
        return;
    }

    sbv_ota_msg_slave_handler.state             = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_slave_handler.next_state        = SBV_OTA_STATE_START;
    sbv_ota_msg_slave_handler.is_updating       = SBV_FALSE;
    sbv_ota_msg_slave_handler.is_update_enable  = SBV_TRUE;

    sbv_ota_msg_slave_handler.data_queue = sbv_cqbuff_create (SBV_OTA_SLAVE_RCV_BUFFER_SIZE, 1);
    if (! sbv_ota_msg_slave_handler.data_queue) {
        LOG_ERROR ("Failed to allocate memory for the Slave FSM task data queue");
        return;
    }

    sbv_ota_msg_slave_handler.rx_queue   = ipc->to_slave_fsm;
    sbv_ota_msg_slave_handler.tx_queue   = ipc->to_installer;

    sbv_rtos_mutex_create (sbv_ota_msg_slave_handler.mu);

    LOG_INFO ("Initializating the OTA Slave FSM task ...");

    sbv_rtos_task_create_static(sbv_task_ota_update_fw_slave, "ota_slave", STACK_SIZE_BASE * 4,
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

    LOG_INFO ("Moving back to IDLE state, resetting the OTA Slave FSM task stack...");
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
    LOG_INFO ("OTA Slave FSM task is starting up...");

    for(;;)
    {
        // Waiting for msg from Master FSM on Master device on every state
        // Blocking for receiving the cmd start msg from Master
        sbv_ota_slave_fsm_handle_state (NULL);
    }
}

void
sbv_ota_slave_fsm_handle_state (void *data)
{
    int ret;
    sbv_ota_state_t current_state, next_state;

    sbv_rtos_mutex_lock (sbv_ota_msg_slave_handler.mu);

    current_state = sbv_ota_slave_fsm_get_current_state();
    next_state    = sbv_ota_slave_fsm_get_next_state();

    // The firmware installer is going to resetting the device
    // Blocking from any further processing of Slave FSM task
    if (! sbv_ota_slave_fsm_is_update_enable ()) {
        LOG_ERROR ("OTA update is disabled, aborting to run OTA Slave FSM process");
        sbv_rtos_mutex_unlock (sbv_ota_msg_slave_handler.mu);

        sbv_rtos_task_delay (SBV_RTOS_MAX_DELAY);
        return;
    }

    // Reset when transit from any other state to IDLE
    if (current_state != SBV_OTA_STATE_IDLE
        && next_state == SBV_OTA_STATE_IDLE) {
        sbv_ota_slave_fsm_reset ();

        goto EXIT;
    }

    ret = sbv_ota_fsm_handle_state (sbv_ota_slave_fsm_state,
                                    current_state, next_state, data);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed executing state transition cb from current state %s to next state %s",
                   sbv_ota_fsm_state_to_string(current_state),
                   sbv_ota_fsm_state_to_string(next_state));

        sbv_ota_slave_fsm_reset ();
        goto EXIT;
    }
    sbv_ota_msg_slave_handler.state = next_state;

EXIT:
    sbv_rtos_mutex_unlock (sbv_ota_msg_slave_handler.mu);
}




int
sbv_ota_slave_fsm_idle (sbv_ota_state_t current_state, void *data)
{
    /* Do nothing */
    return SBV_OK;
}

int
sbv_ota_slave_fsm_start (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t resp_type;

    if (current_state != SBV_OTA_STATE_IDLE)
    {
        LOG_ERROR ("Invalid current state %s is not IDLE, transitting back to IDLE",
                    sbv_ota_fsm_state_to_string(current_state));
        return SBV_ERROR;
    }

    ret = sbv_ota_msg_fsm_handle_cmd(sbv_ota_msg_slave_handler.data_queue,
                                     &(sbv_ota_msg_slave_handler.peer_seq_num),
                                     SBV_OTA_CMD_START, SBV_RTOS_MAX_DELAY_MS);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to handle cmd start packet, sending NACK to OTA Master FSM");
    } else if (! sbv_ota_msg_slave_handler.is_updating) {
        sbv_ota_msg_slave_handler.is_updating = SBV_TRUE;

        LOG_INFO ("Recv OTA cmd Start, Slave FSM start update process...");
    }

    resp_type = (ret == SBV_OK) ? SBV_OTA_ACK : SBV_OTA_NACK;

    // Only increase the sequence number when receiving a valid cmd Start packet
    sbv_ota_msg_slave_handler.seq_num += ((ret == SBV_OK) ? \
                                            SBV_OTA_RESP_PACKET_LEN : 0);

    sbv_ota_send_resp_with_retry (resp_type, sbv_ota_msg_slave_handler.seq_num,
                                  SBV_OTA_SLAVE_MSG_MAX_RETRY,
                                  SBV_OTA_SLAVE_MSG_TIMEOUT_MS);

    sbv_ota_msg_slave_handler.next_state = SBV_OTA_SLAVE_NEXT_STATE(SBV_OTA_STATE_START,
                                                                    SBV_OTA_STATE_HEADER,
                                                                    ret, resp_type);

    return SBV_OK;
}

int
sbv_ota_slave_fsm_header (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t resp_type;

    if (current_state != SBV_OTA_STATE_START
        || ! sbv_ota_slave_fsm_is_updating())
    {
        LOG_ERROR ("Invalid state transition, current state %s, \
                    required state START, is updating %s, transitting back to IDLE",
                    sbv_ota_fsm_state_to_string(current_state),
                    sbv_ota_slave_fsm_is_updating() ? "True" : "False");
        return SBV_ERROR;
    }

    ret = sbv_ota_msg_fsm_handle_header(sbv_ota_msg_slave_handler.data_queue,
                                        &(sbv_ota_msg_slave_handler.peer_seq_num),
                                        SBV_RTOS_MAX_DELAY_MS,
                                        &(sbv_ota_msg_slave_handler.new_fw_metadata));
    if (ret != SBV_OK) {
        if (ret == SVB_OTA_SEQ_DUP) {
            LOG_WARN ("Received duplication cmd start packet, sending ACK to OTA Master FSM");
        } else {
            LOG_ERROR ("Failed to handle header packet, sending NACK to OTA Master FSM");
        }
    }

    // Send ACK when header pkt is valid or when we receive again the start cmd packet
    // in the last state, since the master may not received our last response packet
    // and still retrying on sending that msg to us
    resp_type = (ret == SBV_OK || ret == SVB_OTA_SEQ_DUP) ? SBV_OTA_ACK : SBV_OTA_NACK;

    // Only increase the sequence number when receiving a valid Header packet
    sbv_ota_msg_slave_handler.seq_num += ((ret == SBV_OK) ? \
                                            SBV_OTA_RESP_PACKET_LEN : 0);

    sbv_ota_send_resp_with_retry (resp_type, sbv_ota_msg_slave_handler.seq_num,
                                  SBV_OTA_SLAVE_MSG_MAX_RETRY,
                                  SBV_OTA_SLAVE_MSG_TIMEOUT_MS);

    sbv_ota_msg_slave_handler.next_state = SBV_OTA_SLAVE_NEXT_STATE(SBV_OTA_STATE_HEADER,
                                                                    SBV_OTA_STATE_DATA,
                                                                    ret, resp_type);

    return SBV_OK;
}

int
sbv_ota_slave_fsm_data (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t resp_type;
    uint8_t* fw_image_head;

    if (current_state != SBV_OTA_STATE_HEADER
        || ! sbv_ota_slave_fsm_is_updating())
    {
        LOG_ERROR ("Invalid state transition, current state %s, \
                    required state HEADER, is updating %s, transitting back to IDLE",
                    sbv_ota_fsm_state_to_string(current_state),
                    sbv_ota_slave_fsm_is_updating() ? "True" : "False");
        return SBV_ERROR;
    }

    do {
        fw_image_head = sbv_ota_msg_slave_handler.fw_image +  \
                            sbv_ota_msg_slave_handler.current_rcv_image_size;
        ret = sbv_ota_msg_fsm_handle_data(sbv_ota_msg_slave_handler.data_queue,
                                          &(sbv_ota_msg_slave_handler.peer_seq_num),
                                          SBV_RTOS_MAX_DELAY_MS, fw_image_head,
                                          sbv_ota_msg_slave_handler.current_rcv_image_size,
                                          sbv_ota_msg_slave_handler.new_fw_metadata.fw_size);
        if (ret != SBV_BUSY && ret != SBV_OK) {
            if (ret == SVB_OTA_SEQ_DUP) {
                LOG_WARN ("Received duplication packet, sending ACK to OTA Master FSM");
            } else {
                LOG_ERROR ("Failed to handle data packet, sending NACK to OTA Master FSM");
            }
        }

        resp_type = (ret == SBV_OK || ret == SBV_BUSY \
                        || ret == SVB_OTA_SEQ_DUP) ? SBV_OTA_ACK : SBV_OTA_NACK;

        // Only increase the sequence number when receiving valid Data packet
        sbv_ota_msg_slave_handler.seq_num += ((ret == SBV_OK \
                                                || ret == SBV_BUSY) ? SBV_OTA_RESP_PACKET_LEN : 0);

        sbv_ota_send_resp_with_retry (resp_type, sbv_ota_msg_slave_handler.seq_num,
                                      SBV_OTA_SLAVE_MSG_MAX_RETRY,
                                      SBV_OTA_SLAVE_MSG_TIMEOUT_MS);
        // Not yet receive all the firmware image, stay at the current DATA state
    } while (ret == SBV_BUSY || ret == SVB_OTA_SEQ_DUP);

    sbv_ota_msg_slave_handler.next_state = SBV_OTA_SLAVE_NEXT_STATE(SBV_OTA_STATE_DATA,
                                                                    SBV_OTA_STATE_END,
                                                                    ret, resp_type);

    return SBV_OK;
}

int
sbv_ota_slave_validate_rcv_fw_image_crc (void)
{
    uint32_t real_fw_img_crc = sbv_ota_calculate_crc(sbv_ota_msg_slave_handler.fw_image,
                                                     sbv_ota_msg_slave_handler.current_rcv_image_size);
    return (real_fw_img_crc == sbv_ota_msg_slave_handler.new_fw_metadata.fw_crc);
}

int
sbv_ota_slave_fsm_end (sbv_ota_state_t current_state, void *data)
{
    int ret;
    uint8_t resp_type;
    sbv_ota_upd_status upd_status;

    if (current_state != SBV_OTA_STATE_DATA
        || ! sbv_ota_slave_fsm_is_updating())
    {
        LOG_ERROR ("Invalid state transition, current state %s, \
                    required state HEADER, is updating %s, transitting back to IDLE",
                    sbv_ota_fsm_state_to_string(current_state),
                    sbv_ota_slave_fsm_is_updating() ? "True" : "False");
        return SBV_ERROR;
    }

    ret = sbv_ota_msg_fsm_handle_cmd(sbv_ota_msg_slave_handler.data_queue,
                                     &(sbv_ota_msg_slave_handler.peer_seq_num),
                                     SBV_OTA_CMD_END, SBV_RTOS_MAX_DELAY_MS);
    if (ret != SBV_OK) {
        if (ret == SVB_OTA_SEQ_DUP) {
            LOG_WARN ("Received duplication data packet, sending ACK to OTA Master FSM");
            goto SEND_RESP;
        } else {
            LOG_ERROR ("Failed to handle cmd end packet, sending Error report to OTA Master FSM");
            goto SEND_REPORT;
        }
    }

    // Verify the image integrity
    if (sbv_ota_slave_validate_rcv_fw_image_crc () != SBV_TRUE) {
        LOG_ERROR ("Failed to validate fw image CRC, aborting fw update process");
        ret = SBV_ERROR;
        goto SEND_REPORT;
    }

    ret = sbv_ota_slave_fsm_fw_installer_tasks_process ();
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to process fw installer control task process");
    }

SEND_REPORT:
    upd_status = (ret == SBV_OK) ? SBV_OTA_UPD_SUCCESS : SBV_OTA_UDP_FAILED;

    sbv_ota_msg_slave_handler.seq_num += ((ret == SBV_OK) ? SBV_OTA_REP_PACKET_LEN : 0);

    sbv_ota_send_report_with_retry (upd_status, 
                                    &(sbv_ota_msg_slave_handler.new_fw_metadata),
                                    sbv_ota_msg_slave_handler.seq_num,
                                    SBV_OTA_SLAVE_MSG_MAX_RETRY,
                                    SBV_OTA_SLAVE_MSG_TIMEOUT_MS);

    sbv_ota_msg_slave_handler.next_state  = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_slave_handler.is_updating = SBV_FALSE;
    return SBV_OK;

SEND_RESP:
    resp_type = (ret == SVB_OTA_SEQ_DUP) ? SBV_OTA_ACK : SBV_OTA_NACK;

    sbv_ota_send_resp_with_retry (resp_type, sbv_ota_msg_slave_handler.seq_num,
                                  SBV_OTA_SLAVE_MSG_MAX_RETRY,
                                  SBV_OTA_SLAVE_MSG_TIMEOUT_MS);

    sbv_ota_msg_slave_handler.next_state = SBV_OTA_STATE_END;

    return SBV_OK;
}

int
sbv_ota_send_system_msg_udp_start (sbv_rtos_queue_handle_t queue, void* img_metadata, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_UDP_START, img_metadata, timeout_ms);
}

int
sbv_ota_send_system_msg_image_write (sbv_rtos_queue_handle_t queue, void* img, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_IMG_WRITE, img, timeout_ms);
}

int
sbv_ota_send_system_msg_udp_finalize (sbv_rtos_queue_handle_t queue, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_UDP_FINALIZE, NULL, timeout_ms);
}

int
sbv_ota_rcv_system_msg (sbv_rtos_queue_handle_t queue, sbv_ota_system_msg_t* system_msg, uint16_t timeout_ms) {
    sbv_rtos_base_type_t status;

    if (system_msg == NULL) {
        LOG_ERROR ("Rcv empty system message from fw installer task, no further processing");
        return SBV_ERROR;
    }

    status = sbv_rtos_queue_rcv (queue, system_msg, sbv_rtos_ms_to_tick (timeout_ms));
    if (status != SBV_RTOS_TRUE) {
        LOG_ERROR ("Failed to rcv system message from fw installer task");
        return SBV_ERROR;
    }

    return SBV_OK;
}

int
sbv_ota_slave_fsm_system_msg_handle (void) {
    int ret;
    sbv_ota_system_msg_t system_msg;

    memset (&system_msg, 0, sizeof (sbv_ota_system_msg_t));

    ret = sbv_ota_rcv_system_msg (sbv_ota_msg_slave_handler.rx_queue,
                                  &system_msg, SBV_OTA_SLAVE_SYSTEM_MSG_RX_TIMEOUT_MS);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to rcv system message from fw installer task, abort system msg handling");
        return ret;
    }

    switch (system_msg.event)
    {
    case SBV_OTA_EVENT_ACK:
        LOG_DEBUG ("Rcv system msg ACK from fw installer task");
        return SBV_OK;
    case SBV_OTA_EVENT_ABORT:
        LOG_DEBUG ("Rcv system msg ABORT from fw installer task");
        return SBV_ERROR;
    default:
        LOG_DEBUG ("Rcv unknown system msg from fw installer task, event=%u", system_msg.event);
        return SBV_ERROR;
    }
}

int
sbv_ota_slave_fsm_start_fw_update (void) {
    int ret;

    ret = sbv_ota_send_system_msg_udp_start (sbv_ota_msg_slave_handler.tx_queue,
                                            &(sbv_ota_msg_slave_handler.new_fw_metadata),
                                            SBV_OTA_SLAVE_SYSTEM_MSG_TX_TIMEOUT_MS);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to send system msg start update process request to fw installer task");
        return ret;
    }

    ret = sbv_ota_slave_fsm_system_msg_handle ();
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to handle the response message from fw installer task");
        return ret;
    }

    return SBV_OK;
}

int
sbv_ota_slave_fsm_send_img_to_installer (void) {
    int ret;

    ret = sbv_ota_send_system_msg_image_write (sbv_ota_msg_slave_handler.tx_queue,
                                               sbv_ota_msg_slave_handler.fw_image,
                                               SBV_OTA_SLAVE_SYSTEM_MSG_TX_TIMEOUT_MS);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to send system msg write fw image request to fw installer task");
        return ret;
    }

    ret = sbv_ota_slave_fsm_system_msg_handle ();
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to handle the response message from fw installer task");
        return ret;
    }

    return SBV_OK;
}

int
sbv_ota_slave_fsm_stop_fw_update (void) {
    int ret;

    ret = sbv_ota_send_system_msg_udp_finalize (sbv_ota_msg_slave_handler.tx_queue,
                                                SBV_OTA_SLAVE_SYSTEM_MSG_TX_TIMEOUT_MS);
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to send system msg finalize update process request to fw installer task");
        return ret;
    }

    ret = sbv_ota_slave_fsm_system_msg_handle ();
    if (ret != SBV_OK) {
        LOG_ERROR ("Failed to handle the response message from fw installer task");
        return ret;
    }

    return SBV_OK;
}

