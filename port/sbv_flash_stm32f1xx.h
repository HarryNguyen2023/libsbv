#ifndef SBV_FLASH_STM32F1XX_H
#define SBV_FLASH_STM32F1XX_H

#include "sbv.h"

int
sbv_flash_stm32f1xx_erase_page(uint32_t page_addr, uint16_t pages_num);
int
sbv_flash_stm32f1xx_write_page (uint8_t *data, uint32_t data_length, uint32_t page_addr);

#endif /* SBV_FLASH_STM32F1XX_H */