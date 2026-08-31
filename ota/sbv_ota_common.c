#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_flash.h"
#include "sbv_ota_common.h"

#ifdef SBV_HW_CRC
#ifdef STM32F1xx
extern CRC_HandleTypeDef hcrc;
#elif defined ESP32xx_IDF
#include "esp_crc.h"
#endif /*STM32F1xx*/
#endif /* SBV_HW_CRC */

volatile sbv_ota_general_cfg *sbv_ota_cfg_flash = (sbv_ota_general_cfg *)(((volatile uint32_t *)(SBV_OTA_CONFIG_FLASH_ADD)));

int
sbv_ota_erase_flash_data (uint32_t page_addr, uint16_t pages_num)
{
    return sbv_flash_erase_page (page_addr, pages_num);
}

int
sbv_ota_write_flash_data (uint8_t *data, uint32_t data_length, uint32_t page_addr)
{
    int ret = SBV_OK;

    if(! data || data_length == 0)
        return SBV_ERROR;

    data_length = (data_length % 2 == 0) ? data_length : data_length + 1;

    ret = sbv_flash_write_page(data, data_length, page_addr);
    if(ret != SBV_OK)
    {
        // LOG
        return ret;
    }

    return SBV_OK;
}

uint8_t
sbv_ota_get_update_slot (sbv_ota_general_cfg* cfg)
{
    uint8_t data_slot = SBV_OTA_INVALID_SLOT;

    if (! cfg)
        return SBV_OTA_INVALID_SLOT;

    for(uint8_t i = 0; i < SBV_OTA_SLOT_NO; ++i) {
        if(cfg->slot_table[i].is_slot_update) {
            data_slot = i;
            break;
        }
    }

    return (data_slot >= SBV_OTA_SLOT_NO) ? SBV_OTA_INVALID_SLOT : data_slot;
}

uint8_t
sbv_ota_get_active_slot (sbv_ota_general_cfg* cfg)
{
    uint8_t data_slot = SBV_OTA_INVALID_SLOT;

     if (! cfg)
        return SBV_OTA_INVALID_SLOT;

    for(uint8_t i = 0; i < SBV_OTA_SLOT_NO; ++i) {
        if(cfg->slot_table[i].is_slot_active) {
            data_slot = i;
            break;
        }
    }

    return (data_slot >= SBV_OTA_SLOT_NO) ? SBV_OTA_INVALID_SLOT : data_slot;
}

/*
 * @brief: Get the available data slot for saving new firmware
 * @param none
 * @retval uint8_t
 */
uint8_t
sbv_ota_get_available_slot_num (void)
{
	uint8_t data_slot = SBV_OTA_INVALID_SLOT;
	sbv_ota_general_cfg cfg;

    /* Read the configuration in flash memory space */
    memset(&cfg, 0, sizeof(sbv_ota_general_cfg));
	memcpy(&cfg, sbv_ota_cfg_flash, sizeof(sbv_ota_general_cfg));

	/* Looking for the valid slot */
	for(uint8_t i = 0; i < SBV_OTA_SLOT_NO; ++i)
	{
		if(cfg.slot_table[i].is_slot_active == SBV_FALSE)
		{
			data_slot = i;
			break;
		}
	}

	return data_slot;
}

/*
 * @brief: Get the current active firmware metadata
 * @param current_fw_medata: output pointer for the firmware metadata
 * @retval int
 */
int
sbv_ota_get_current_fw_metadata (sbv_ota_fw_metadata_t* current_fw_medata)
{
    int i, found = SBV_FALSE;
	sbv_ota_general_cfg cfg;

    if (! current_fw_medata)
        return -1;

    /* Read the configuration in flash memory space */
    memset(&cfg, 0, sizeof(sbv_ota_general_cfg));
	memcpy(&cfg, sbv_ota_cfg_flash, sizeof(sbv_ota_general_cfg));

    for (i = 0; i < SBV_OTA_SLOT_NO; ++i)
    {
        if (cfg.slot_table[i].is_slot_active)
        {
            memcpy (current_fw_medata, &(cfg.slot_table[i].metadata), sizeof (sbv_ota_fw_metadata_t));
            found = SBV_TRUE;
            break;
        }
    }

    if (! found)
        return -1;

    return 0;
}

/*
 * @brief: Calculate the CRC for the input data buffer
 * @param buffer: input data buffer
 * @param buffer_length: length of the input data buffer
 * @retval uint32_t
 */
uint32_t
sbv_ota_calculate_crc (uint8_t *buffer, uint32_t buffer_length)
{
#ifdef SBV_HW_CRC
#ifdef STM32F1xx
    return HAL_CRC_Calculate(&hcrc, buffer, buffer_length);
#elif defined ESP32xx_IDF
    return esp_crc32_le(0xFFFFFFFF, (uint8_t *)buffer, buffer_length);
#endif /*STM32F1xx*/
#else
    uint32_t crc = 0xFFFFFFFF;

    if(!buffer || !buffer_length)
        return 0;

    for (uint32_t i = 0; i < buffer_length; i++)
    {
        crc ^= *(buffer + i);
        for (uint8_t j = 0; j < 32; j++)
        {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
#endif /*SBV_HW_CRC*/
}

/*
 * @brief: Calculate the HMAC for the firmware image metadata on the fielsystem for validity check
 *          1st, Calculate the CRC of the firmware storage register address
 *          2nd, Calcualte the CRC of the combination of fw version, fw timestampt, and the 1st CRC
 * @param page_addr: firmware storage register address
 * @param fw_metadata: input fw metadata
 * @retval uint32_t
 */
static uint32_t
sbv_ota_fw_crc_cal (const uint32_t page_addr, const sbv_ota_fw_metadata_t fw_metadata)
{
    uint8_t fw_img_data[36] = {0};
    uint32_t fw_img_crc;

    /* Create the first Hash layer */
    fw_img_crc = sbv_ota_calculate_crc((uint8_t *)&page_addr, fw_metadata.fw_size);

    memcpy (fw_img_data, fw_metadata.fw_version, 12);
    memcpy (fw_img_data + 12, fw_metadata.fw_timestamp, 20);
    memcpy (fw_img_data + 32, &fw_img_crc, sizeof(uint32_t));

    /* Create the Second Hash layer */
    fw_img_crc = sbv_ota_calculate_crc (fw_img_data, 36);
    return fw_img_crc;
}

int
sbv_ota_fw_image_crc_validate (const uint32_t page_addr, const sbv_ota_fw_metadata_t fw_metadata)
{
    uint32_t fw_img_crc;

    fw_img_crc = sbv_ota_fw_crc_cal (page_addr, fw_metadata);
    return (fw_img_crc == fw_metadata.fw_crc) ? SBV_TRUE : SBV_FALSE;
}

int
sbv_ota_is_valid_page_addr(const uint32_t slot_pag_addr)
{
    return (slot_pag_addr == SBV_OTA_SLOT0_FLASH_ADD || slot_pag_addr == SBV_OTA_SLOT1_FLASH_ADD);
}

int
sbv_ota_is_valid_fw_slot(uint16_t image_slot)
{
    return (image_slot < SBV_OTA_SLOT_NO && SBV_OTA_SLOT_NO != SBV_OTA_INVALID_SLOT);
}