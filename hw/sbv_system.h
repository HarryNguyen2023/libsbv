#ifndef SBV_SYSTEM_H
#define SBV_SYSTEM_H

typedef struct sbv_system_hw_cb_t {
    int (*sbv_system_get_uid) (uint32_t[]);
    void (*sbv_system_reset) (void);
    void (*sbv_system_application_shift_to_addr) (uint32_t new_app_addr);
    void (*sbv_system_delay) (uint32_t delay_ms);
} sbv_system_hw_cb_t;

int
sbv_system_get_uid (uint32_t uid[]);
void
sbv_system_reset (void);
void
sbv_system_application_shift_to_addr (uint32_t new_app_addr);
void
sbv_system_delay (uint32_t delay_ms);

#endif /* SBV_SYSTEM_H */