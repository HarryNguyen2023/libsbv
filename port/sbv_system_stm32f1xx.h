#ifndef SBV_SYSTEM_STM32F1XX_H
#define SBV_SYSTEM_STM32F1XX_H

uint32_t
sbv_system_stm32f1xx_get_uid (void);
void
sbv_system_stm32f1xx_reset (void);
void
sbv_system_stm32f1xx_application_shift_to_addr (uint32_t new_app_addr);
void
sbv_system_stm32f1xx_delay (uint32_t delay_ms);

#endif /* SBV_SYSTEM_STM32F1XX_H */