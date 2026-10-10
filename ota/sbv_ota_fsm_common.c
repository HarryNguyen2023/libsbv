#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_log.h"
#include "sbv_cqbuff.h"
#include "sbv_ota_common.h"
#include "sbv_ota_msg.h"
#include "sbv_ota_fsm_common.h"

int
sbv_ota_fsm_is_valid_state (sbv_ota_state_t state)
{
    return (state >= SBV_OTA_STATE_IDLE && state < SBV_OTA_STATE_MAX);
}

char *
sbv_ota_fsm_state_to_string (sbv_ota_state_t state) {
    switch (state)
    {
    case SBV_OTA_STATE_IDLE:
        return "Idle";
    case SBV_OTA_STATE_START:
        return "Start";
    case SBV_OTA_STATE_HEADER:
        return "Header";
    case SBV_OTA_STATE_DATA:
        return "Data";
    case SBV_OTA_STATE_END:
        return "End";
    default:
        return "";
    }
}

int
sbv_ota_fsm_handle_state (struct sbv_ota_fsm_cb_t state_table[][SBV_OTA_STATE_MAX], 
                          sbv_ota_state_t current_state, sbv_ota_state_t next_state, void *data)
{
    struct sbv_ota_fsm_cb_t *state_cb;

    if (! (sbv_ota_fsm_is_valid_state (current_state))
        || ! (sbv_ota_fsm_is_valid_state (next_state)))
        return SBV_ERROR;

    state_cb = (*(state_table + current_state)) + next_state;
    if (! state_cb || ! (state_cb->state_func))
    {
        LOG_ERROR ("FSM transit state cb function is nil, current state=%s, next state=%s",
                    sbv_ota_fsm_state_to_string(current_state),
                    sbv_ota_fsm_state_to_string(next_state));
        return SBV_ERROR;
    }

    return (*state_cb->state_func) (current_state, data);
}