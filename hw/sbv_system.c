#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_system.h"
#ifdef STM32F1xx
#include "sbv_system_stm32f1xx.h"
#endif /* STM32F1xx */

sbv_system_hw_cb_t sbv_system_hw_cb = {
#ifdef STM32F1xx
    .sbv_system_get_uid = sbv_system_stm32f1xx_get_uid,
    .sbv_system_reset   = sbv_system_stm32f1xx_reset,
    .sbv_system_application_shift_to_addr = sbv_system_stm32f1xx_application_shift_to_addr,
    .sbv_system_delay   = sbv_system_stm32f1xx_delay,
#endif /* STM32F1xx */
};

uint32_t
sbv_system_get_uid (void) {
    if (sbv_system_hw_cb.sbv_system_get_uid) {
        return (sbv_system_hw_cb.sbv_system_get_uid) ();
    }

    return 0;
}

void
sbv_system_reset (void) {
    if (sbv_system_hw_cb.sbv_system_reset) {
        return (sbv_system_hw_cb.sbv_system_reset) ();
    }
}

void
sbv_system_application_shift_to_addr (uint32_t new_app_addr) {
    if (sbv_system_hw_cb.sbv_system_application_shift_to_addr) {
        return (sbv_system_hw_cb.sbv_system_application_shift_to_addr) (new_app_addr);
    }
}

void
sbv_system_delay (uint32_t delay_ms) {
    if (sbv_system_hw_cb.sbv_system_delay) {
        return (sbv_system_hw_cb.sbv_system_delay) (delay_ms);
    }
}