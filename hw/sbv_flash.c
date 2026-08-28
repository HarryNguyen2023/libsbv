#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_flash.h"

struct sbv_flash_hw_cb_t sbv_flash_hw_cb = {
#ifdef STM32F1xx
    .sbv_flash_erase_page = sbv_flash_stm32f1xx_erase_page,
    .sbv_flash_write_page = sbv_flash_stm32f1xx_write_page,
#endif
};

int
sbv_flash_erase_page(uint32_t page_addr, uint16_t pages_num)
{
    if (sbv_flash_hw_cb.sbv_flash_erase_page) {
        return (sbv_flash_hw_cb.sbv_flash_erase_page) (page_addr, pages_num);
    }

    return 0;
}

int
sbv_flash_write_page (uint8_t *data, uint32_t data_length, uint32_t page_addr)
{
    if (sbv_flash_hw_cb.sbv_flash_write_page) {
        return (sbv_flash_hw_cb.sbv_flash_write_page) (data, data_length, page_addr);
    }

    return 0;
}