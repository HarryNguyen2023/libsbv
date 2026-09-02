#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_ota_common.h"
#include "sbv_ota_msg.h"
#include "sbv_ota_fsm_common.h"

int
sbv_ota_fsm_is_valid_state (sbv_ota_state_t state)
{
    return (state >= SBV_OTA_STATE_IDLE && state < SBV_OTA_STATE_MAX);
}

void sbv_ota_fsm_handle_state (struct sbv_ota_fsm_cb_t **state_table, 
                               sbv_ota_state_t current_state, sbv_ota_state_t next_state, void *data)
{
    struct sbv_ota_fsm_cb_t *state_cb;

    if (current_state == next_state
        || ! (sbv_ota_fsm_is_valid_state (current_state))
        || ! (sbv_ota_fsm_is_valid_state (next_state)))
        return;

    state_cb = (*(state_table + current_state)) + next_state;
    if (! state_cb)
    {
        // LOG
        return;
    }

    (*state_cb->state_func) (current_state, data);
}