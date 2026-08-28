#ifndef SBV_FLASH_H
#define SBV_FLASH_H

#include "sbv.h"

#ifdef STM32F1xx
#include "sbv_flash_stm32f1xx.h"
#endif

struct sbv_flash_hw_cb_t
{
    int (*sbv_flash_erase_page) (uint32_t, uint16_t);
    int (*sbv_flash_write_page) (uint8_t*, uint32_t, uint32_t);
};

int
sbv_flash_erase_page(uint32_t page_addr, uint16_t pages_num);
int
sbv_flash_write_page (uint8_t *data, uint32_t data_length, uint32_t page_addr);

#endif /*SBV_FLASH_H*/