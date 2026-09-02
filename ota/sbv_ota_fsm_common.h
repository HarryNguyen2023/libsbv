#ifndef SBV_OTA_FSM_COMMON_H
#define SBV_OTA_FSM_COMMON_H

struct sbv_ota_fsm_cb_t {
    sbv_ota_state_t next_state;
    void            (*state_func) (sbv_ota_state_t, void *);
};

int
sbv_ota_fsm_is_valid_state (sbv_ota_state_t state);

void
sbv_ota_fsm_handle_state (struct sbv_ota_fsm_cb_t **state_table, 
                          sbv_ota_state_t current_state, sbv_ota_state_t next_state, void *data);

#endif /* SBV_OTA_FSM_COMMON_H */