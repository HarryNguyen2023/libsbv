#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_flash.h"
#include "sbv_system.h"
#include "sbv_ota_common.h"

#ifdef SBV_HW_CRC
#ifdef STM32F1xx
extern CRC_HandleTypeDef hcrc;
#elif defined ESP32xx_IDF
#include "esp_crc.h"
#endif /*STM32F1xx*/
#endif /* SBV_HW_CRC */

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

void
sbv_ota_cfg_read (sbv_ota_general_cfg_t *c) {
    if (! c) {
        // LOG
        return;
    }
    memset (c, 0, sizeof (sbv_ota_general_cfg_t));
    memcpy (c, (const void *)((volatile uint32_t *)SBV_OTA_CONFIG_FLASH_ADD), sizeof (sbv_ota_general_cfg_t));
}

int
sbv_cfg_validate (const sbv_ota_general_cfg_t *c) {
    uint32_t metadata_crc, new_crc;

    if (c->magic != SBV_OTA_CONFIG_FLASH_ADD) {
        // LOG
        return SBV_ERROR;
    }

    metadata_crc = c->crc;
    new_crc      = sbv_ota_calculate_crc ((uint8_t *)c, offsetof(sbv_ota_general_cfg_t, crc));

    return (new_crc == metadata_crc) ? SBV_OK : SBV_ERROR;
}

int
sbv_ota_cfg_read_and_validate (sbv_ota_general_cfg_t *c) {
    if (! c) {
        // LOG
        return -1;
    }

    sbv_ota_cfg_read (c);

    return sbv_cfg_validate (c);
}

int
sbv_ota_cfg_commit(sbv_ota_general_cfg_t *c) {
    int ret;
    uint32_t metadata_crc;

    if (! c) {
        // LOG
        return SBV_ERROR;
    }

    c->magic = SBV_OTA_CONFIG_FLASH_ADD;
    metadata_crc = sbv_ota_calculate_crc ((uint8_t *)c, offsetof(sbv_ota_general_cfg_t, crc));
    c->crc   = metadata_crc;

    ret = sbv_ota_erase_flash_data (SBV_OTA_CONFIG_FLASH_ADD, SBV_OTA_GEN_CFG_PAGES);
    if (ret != SBV_OK)
    {
        /* LOG */
        return SBV_ERROR;
    }

    ret = sbv_ota_write_flash_data((uint8_t *)c, sizeof(sbv_ota_general_cfg_t), SBV_OTA_CONFIG_FLASH_ADD);
    if (ret != SBV_OK)
    {
        /* LOG */
        return SBV_ERROR;
    }

    return SBV_OK;
}

uint8_t
sbv_ota_get_update_slot (sbv_ota_general_cfg_t* cfg)
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
sbv_ota_get_active_slot (sbv_ota_general_cfg_t* cfg)
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
    int ret;
	uint8_t data_slot = SBV_OTA_INVALID_SLOT;
	sbv_ota_general_cfg_t cfg;

    /* Read the configuration in flash memory space */
    ret = sbv_ota_cfg_read_and_validate (&cfg);
    if (ret != SBV_OK) {
        // LOG
        return SBV_OTA_INVALID_SLOT;
    }

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
    int i, found = SBV_FALSE, ret;
	sbv_ota_general_cfg_t cfg;

    if (! current_fw_medata)
        return -1;

    /* Read the configuration in flash memory space */
    ret = sbv_ota_cfg_read_and_validate (&cfg);
    if (ret != SBV_OK) {
        // LOG
        return -1;
    }

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
sbv_ota_calculate_crc (const uint8_t *buffer, const uint32_t buffer_length)
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

uint32_t
sbv_ota_frame_crc (uint8_t *pkt, uint32_t pkt_length) {
    return sbv_ota_calculate_crc (pkt, pkt_length);
}

uint32_t
sbv_ota_img_crc (const uint8_t *img, uint32_t img_size) {
    if (! img || img_size == 0 || img_size > SBV_OTA_SLOT_MAX_SIZE) {
        // LOG
        return 0;
    }
    return sbv_ota_calculate_crc (img, img_size);
}

/*
 * @brief: Calculate the CRC for the firmware image on the flash for validity check
 * @param page_addr: firmware storage register address
 * @param fw_metadata: input fw metadata
 * @retval int
 */
int
sbv_ota_fw_image_crc_validate (const uint32_t page_addr, const sbv_ota_fw_metadata_t fw_metadata)
{
    uint32_t fw_img_crc;
    const uint8_t *img;

    if (! sbv_ota_is_valid_page_addr(page_addr)) {
        // LOG
        return SBV_ERROR;
    }

    if (fw_metadata.fw_size == 0 || fw_metadata.fw_size > SBV_OTA_SLOT_MAX_SIZE) {
        // LOG
        return SBV_ERROR;
    }

    img = (const uint8_t *)page_addr;
    fw_img_crc = sbv_ota_img_crc((uint8_t *)img, fw_metadata.fw_size);

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

int
sbv_ota_send_system_msg (sbv_rtos_queue_handle_t queue, sbv_ota_system_msg_event_t event,
                         void *data, uint16_t timeout_ms) {
    sbv_rtos_base_type_t status;
    sbv_ota_system_msg_t msg;
    uint16_t tick_to_wait;

    tick_to_wait = sbv_rtos_ms_to_tick (timeout_ms);
    memset(&msg, 0, sizeof(sbv_ota_system_msg_t));

    msg.event = event;
    msg.data  = data;
    status = sbv_rtos_queue_send (queue, &msg, tick_to_wait);
    if (status != SBV_RTOS_TRUE) {
        // LOG
        return -1;
    }

    return SBV_OK;
}

void
sbv_ota_random_init_unique(void)
{
    uint32_t *uid, system_uid;

    system_uid = sbv_system_get_uid();
    uid = (uint32_t *)system_uid;

    // Mix the 96-bit UID into a 32-bit seed
    uint32_t seed = uid[0] ^ uid[1] ^ uid[2];
    seed ^= (uid[0] << 11) | (uid[1] >> 7);

    srand(seed);
}

uint16_t
sbv_ota_get_random_seq_number (void) {
    return (rand() % 0xFFFF);
}

int
sbv_ota_seq_num_validate (uint16_t* curr_seq_num, uint16_t new_seq_num, uint16_t seq_num_offset) {
    int ret;
    uint16_t expected_seq_num;

    if (! curr_seq_num) {
        // LOG
        return SBV_ERROR;
    }

    expected_seq_num = *curr_seq_num + seq_num_offset;
        
    if (new_seq_num == expected_seq_num) {
        *curr_seq_num = expected_seq_num;
        return SBV_OK;
    }
    
    if (new_seq_num == *curr_seq_num) {
        // LOG
        return SVB_OTA_SEQ_DUP;
    }

    return SBV_ERROR;
}