#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_flash_stm32f1xx.h"

int
sbv_flash_stm32f1xx_erase_page(uint32_t page_addr, uint16_t pages_num)
{
    int ret = SBV_OK;

    ret = HAL_FLASH_Unlock();
    if(ret != SBV_OK)
        return ret;

    FLASH_EraseInitTypeDef erase_init_struct;
    uint32_t erase_error;

    erase_init_struct.TypeErase     = FLASH_TYPEERASE_PAGES;
    erase_init_struct.PageAddress   = page_addr;
    erase_init_struct.NbPages       = pages_num;
    ret = HAL_FLASHEx_Erase(&erase_init_struct, &erase_error);
    if (ret != SBV_OK)
    {
        // LOG
    }

    HAL_FLASH_Lock();
    return ret;
}


int
sbv_flash_stm32f1xx_write_page (uint8_t *data, uint32_t data_length, uint32_t page_addr)
{
    int ret = SBV_OK;
    uint16_t halfword_data;
    uint32_t ota_half_word_addr = page_addr;

    if(! data || data_length == 0)
        return SBV_ERROR;

    ret = HAL_FLASH_Unlock();
    if(ret != SBV_OK)
        return ret;

    for(uint32_t i = 0; i < data_length; i += 2)
    {
        halfword_data = ((*(data + i + 1)) << 8) | (*(data + i));
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ota_half_word_addr, halfword_data);
        if(ret != SBV_OK)
            goto EXIT_ERR;
        ota_half_word_addr += 2;
    }

    HAL_FLASH_Lock();
    return SBV_OK;

EXIT_ERR:
    // LOG
    HAL_FLASH_Lock();
    return SBV_ERROR;
}