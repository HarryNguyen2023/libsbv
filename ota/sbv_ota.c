#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_gpio.h"
#include "sbv_cqbuff.h"
#include "sbv_ota.h"
#include "sbv_ota_msg.h"

#define SBV_OTA_TX_MAX_RETRY            (5)
#define SBV_OTA_UPDATE_TX_MAX_WAIT_MS   (10)
#define SBV_OTA_LOAD_NEW_FW_APP_WAIT_MS (10 * 1000)
#define SBV_OTA_SEND_UPDATE_FW_PRIO     (3)
#define SBV_OTA_UPDATE_FW_PRIO          (3)

sbv_rtos_mutex_t SBV_OTA_DB_MUTEX;

sbv_event_group_handle_t    sbv_ota_event_group;
sbv_rtos_task_handle_t      sbv_ota_send_update_fw_handle;
sbv_rtos_task_handle_t      sbv_ota_update_fw_handle;

volatile sbv_ota_general_cfg *sbv_ota_cfg_flash = (sbv_ota_general_cfg *)(((volatile uint32_t *)(SBV_OTA_CONFIG_FLASH_ADD)));

extern sbv_ota_msg_tx_instance_t sbv_ota_msg_tx_instance;
extern sbv_ota_msg_rx_instance_t sbv_ota_msg_rx_instance;

extern sbv_ota_msg_hw_cb_t sbv_ota_msg_hw_cb;

sbv_ota_fw_metadata_t fw_metadata;
uint8_t fw_pages[SBV_OTA_PAGES_SIZE];

void sbv_ota_update_fw_thread (void *param);

void
sbv_ota_update_init(void)
{
    sbv_ota_event_group = sbv_rtos_event_group_create();
    sbv_rtos_mutex_create(SBV_OTA_DB_MUTEX);

    memset(&sbv_ota_msg_rx_instance, 0, sizeof(sbv_ota_msg_rx_instance_t));
    sbv_rtos_mutex_create (sbv_ota_msg_rx_instance.mutex);
    sbv_ota_msg_rx_instance.rx_state = SBV_OTA_STATE_IDLE;
    sbv_ota_msg_rx_instance.rx_queue = sbv_cqbuff_create (SBV_OTA_PAGES_SIZE, sizeof (uint8_t));
    if (! sbv_ota_msg_rx_instance.rx_queue)
    {
        /* LOG */
        return;
    }

    sbv_rtos_task_create(sbv_ota_update_fw_thread, "update_fw", 512,
                         NULL, SBV_OTA_UPDATE_FW_PRIO, &sbv_ota_update_fw_handle);
}

void
sbv_ota_send_update_fw_thread (void* param)
{

    /* Register for hw callback reception */
    if (! sbv_ota_msg_hw_cb.sbv_ota_reg_cb)
    {
        /* LOG */
        return;
    }

    sbv_ota_msg_hw_cb.sbv_ota_reg_cb(sbv_ota_msg_resp_handle);

    memset(&sbv_ota_msg_tx_instance, 0, sizeof (sbv_ota_msg_tx_instance_t));
    sbv_ota_msg_tx_instance.max_retry = SBV_OTA_TX_MAX_RETRY;
    sbv_ota_msg_tx_instance.tx_state  = SBV_OTA_STATE_IDLE;

    for(;;)
    {
        if (sbv_ota_msg_tx_instance.is_updating)
        {
            sbv_ota_handle_state (sbv_ota_msg_tx_instance.tx_state,
                                  sbv_ota_msg_tx_instance.next_state, NULL);
        }
        else
        {
            /* Wait for some event to trigger update */
            {
                sbv_ota_msg_tx_instance.is_updating = SBV_TRUE;
                sbv_ota_msg_tx_instance.next_state  = SBV_OTA_STATE_START;
            }
        }
    }
}

uint32_t
sbv_ota_fw_crc_cal (const uint32_t page_add, const sbv_ota_fw_metadata_t fw_metadata)
{
    uint8_t fw_img_data[36] = {0};
    uint32_t fw_img_crc;

    /* Create the first Hash layer */
    fw_img_crc = sbv_ota_msg_crc_calculate((uint8_t *)page_add, fw_metadata.fw_size);

    memcpy (fw_img_data, fw_metadata.fw_version, 12);
    memcpy (fw_img_data + 12, fw_metadata.fw_timestamp, 20);
    memcpy (fw_img_data + 32, &fw_img_crc, 4);

    /* Create the Second Hash layer */
    fw_img_crc = sbv_ota_msg_crc_calculate (fw_img_data, 36);
    return fw_img_crc;
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

    SBV_OTA_DB_MUTEX_LOCK;
    /* Read the configuration in flash memory space */
	memcpy(&cfg, sbv_ota_cfg_flash, sizeof(sbv_ota_general_cfg));

	/* Looking for the valid slot */
	for(uint8_t i = 0; i < SBV_OTA_SLOT_NO; ++i)
	{
		if(cfg.slot_table[i].is_slot_active == 0)
		{
			data_slot = i;
			break;
		}
	}

    SBV_OTA_DB_MUTEX_UNLOCK;

	return data_slot;
}

static int
sbv_ota_erase_flash_pages (uint32_t page_addr, uint16_t pages_num)
{
    int ret = SBV_OK;

#ifdef STM32F1xx
    ret = HAL_FLASH_Unlock();
    if(ret != SBV_OK)
        return ret;

    FLASH_EraseInitTypeDef erase_init_struct;
    uint32_t erase_error;

    erase_init_struct.TypeErase     = FLASH_TYPEERASE_PAGES;
    erase_init_struct.PageAddress   = page_addr;
    erase_init_struct.NbPages       = pages_num;
    ret = HAL_FLASHEx_Erase(&erase_init_struct, &erase_error);

    HAL_FLASH_Lock();
    return ret;
#else
    return ret;
#endif /*STM32F1xx*/
}

/*
 * @brief: Get the available data slot for firmware update
 * @param none
 * @retval uint8_t
 */
static uint8_t
sbv_ota_get_available_fw_update_slot (void)
{
    int ret;
	uint8_t data_slot = SBV_OTA_INVALID_SLOT, is_image_valid, is_update_cfg = SBV_FALSE;
    uint32_t fw_img_crc, slot_page_addr;
    sbv_ota_general_cfg cfg;

	/* Read the configuration in flash memory space */
	memcpy(&cfg, sbv_ota_cfg_flash, sizeof(sbv_ota_general_cfg));

	/* Looking for the new update slot and boot it up */
    if (cfg.reboot_reason == SBV_OTA_NEW_UPDATE_BOOT)
    {
        for(uint8_t i = 0; i < SBV_OTA_SLOT_NO; ++i)
        {
            if(cfg.slot_table[i].is_slot_update)
            {
                data_slot = i;
                break;
            }
        }

        if (data_slot >= SBV_OTA_SLOT_NO
            || data_slot == SBV_OTA_INVALID_SLOT)
            return SBV_OTA_INVALID_SLOT;

        /* Perform checking of the new fw image signature */
        slot_page_addr = SBV_OTA_SLOT_PAGE_ADDR (data_slot);
        fw_img_crc = sbv_ota_fw_crc_cal (slot_page_addr, cfg.slot_table[data_slot].metadata);
        is_image_valid = (fw_img_crc == cfg.slot_table[data_slot].metadata.fw_crc) ? SBV_TRUE : SBV_FALSE;

        is_update_cfg = SBV_TRUE;
        if (is_image_valid)
        {
            for(uint8_t i = 0; i < SBV_OTA_SLOT_NO; ++i)
            {
                if(i == data_slot)
                {
                    cfg.slot_table[i].is_slot_valid     = SBV_TRUE;
                    cfg.slot_table[i].is_slot_active    = SBV_TRUE;
                    cfg.slot_table[i].is_slot_update    = SBV_FALSE;
                }
                else
                {
                    cfg.slot_table[i].is_slot_valid     = SBV_TRUE;
                    cfg.slot_table[i].is_slot_active    = SBV_FALSE;
                    cfg.slot_table[i].is_slot_update    = SBV_FALSE;
                }
            }
        }
        else
        {
            /* LOG */
            for(uint8_t i = 0; i < SBV_OTA_SLOT_NO; ++i)
            {
                if(i == data_slot)
                {
                    cfg.slot_table[i].is_slot_valid     = SBV_FALSE;
                    cfg.slot_table[i].is_slot_active    = SBV_FALSE;
                    cfg.slot_table[i].is_slot_update    = SBV_FALSE;
                }
            }
            goto FALL_BACK;
        }
    }
    else if (cfg.reboot_reason == SBV_OTA_POWER_UP_BOOT)
    {
FALL_BACK:
        for(uint8_t i = 0; i < SBV_OTA_SLOT_NO; ++i)
        {
            if(cfg.slot_table[i].is_slot_active)
            {
                data_slot = i;
                break;
            }
        }

        if (data_slot >= SBV_OTA_SLOT_NO)
            return SBV_OTA_INVALID_SLOT;

        /* Perform checking of the new fw image signature */
        slot_page_addr = SBV_OTA_SLOT_PAGE_ADDR (data_slot);
        fw_img_crc = sbv_ota_fw_crc_cal (slot_page_addr, cfg.slot_table[data_slot].metadata);
        is_image_valid = (fw_img_crc == cfg.slot_table[data_slot].metadata.fw_crc) ? SBV_TRUE : SBV_FALSE;
        if (! is_image_valid)
        {
            cfg.slot_table[data_slot].is_slot_valid     = SBV_FALSE;
            cfg.slot_table[data_slot].is_slot_active    = SBV_FALSE;
            cfg.slot_table[data_slot].is_slot_update    = SBV_FALSE;
            is_update_cfg   = SBV_TRUE;
            data_slot       = SBV_OTA_INVALID_SLOT;
        }
    }

    if (is_update_cfg)
    {
        ret = sbv_ota_erase_flash_pages (SBV_OTA_CONFIG_FLASH_ADD, SBV_OTA_GEN_CFG_PAGES);
        if (ret != SBV_OK)
        {
            /* LOG */
            return SBV_OTA_INVALID_SLOT;
        }

        ret = sbv_ota_write_flash_data((uint8_t *)&cfg, sizeof(sbv_ota_general_cfg), SBV_OTA_CONFIG_FLASH_ADD);
        if (ret != SBV_OK)
        {
            /* LOG */
            return SBV_OTA_INVALID_SLOT;
        }
    }
    
	return data_slot;
}

static int
sbv_ota_write_flash_pages (uint8_t *data, uint32_t data_length, uint32_t page_addr)
{
    int ret = SBV_OK;
    uint16_t halfword_data;
    uint32_t ota_half_word_addr = page_addr;

    if(! data)
        return SBV_ERROR;

    if(data_length % 2 != 0)
        return SBV_ERROR;

#ifdef STM32F1xx
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
    HAL_FLASH_Lock();
    return SBV_ERROR;
#else
    return ret;
#endif /*STM32F1xx*/
}

int
sbv_ota_write_flash_data (uint8_t *data, uint32_t data_length, uint32_t page_addr)
{
    int ret =SBV_OK;

    if(!data)
        return SBV_ERROR;

    if(data_length % 2 != 0)
        data_length += 1;

    ret = sbv_ota_write_flash_pages(data, data_length, page_addr);
    if(ret != SBV_OK)
        return ret;

    return SBV_OK;
}

int
sbv_ota_get_current_fw_metadata (sbv_ota_fw_metadata_t* current_fw_medata)
{
    int i;
	sbv_ota_general_cfg cfg;

    if (! current_fw_medata)
        return -1;

    SBV_OTA_DB_MUTEX_LOCK;

    /* Read the configuration in flash memory space */
	memcpy(&cfg, sbv_ota_cfg_flash, sizeof(sbv_ota_general_cfg));

    for (i = 0; i < SBV_OTA_SLOT_NO; ++i)
    {
        if (cfg.slot_table[i].is_slot_active)
        {
            memcpy (current_fw_medata, &(cfg.slot_table[i].metadata), sizeof (sbv_ota_fw_metadata_t));
            break;
        }
    }

    SBV_OTA_DB_MUTEX_UNLOCK;

    return 0;
}

int
sbv_ota_save_fw_img_cfg (uint16_t image_slot, sbv_ota_fw_metadata_t* slot_metadata, int is_image_valid)
{
    int ret;
	sbv_ota_general_cfg cfg;

    if (image_slot == SBV_OTA_INVALID_SLOT
        || image_slot >= SBV_OTA_SLOT_NO
        || slot_metadata == NULL)
        return -1;

    SBV_OTA_DB_MUTEX_LOCK;

    /* Read the configuration in flash memory space */
	memcpy(&cfg, sbv_ota_cfg_flash, sizeof(sbv_ota_general_cfg));

    if (is_image_valid)
    {
        cfg.reboot_reason = SBV_OTA_NEW_UPDATE_BOOT;
        cfg.slot_table[image_slot].is_slot_active = SBV_FALSE;
        cfg.slot_table[image_slot].is_slot_valid  = SBV_TRUE;
        cfg.slot_table[image_slot].is_slot_update = SBV_TRUE;
        memcpy (&(cfg.slot_table[image_slot].metadata), slot_metadata, sizeof (sbv_ota_fw_metadata_t));
    }
    else
    {
        cfg.reboot_reason = SBV_OTA_POWER_UP_BOOT;
        cfg.slot_table[image_slot].is_slot_active = SBV_FALSE;
        cfg.slot_table[image_slot].is_slot_valid  = SBV_FALSE;
        cfg.slot_table[image_slot].is_slot_update = SBV_FALSE;
        memcpy (&(cfg.slot_table[image_slot].metadata), slot_metadata, sizeof (sbv_ota_fw_metadata_t));
    }

    ret = sbv_ota_erase_flash_pages (SBV_OTA_CONFIG_FLASH_ADD, SBV_OTA_GEN_CFG_PAGES);
    if (ret != SBV_OK)
    {
        /* LOG */
        return -1;
    }

    ret = sbv_ota_write_flash_data((uint8_t *)&cfg, sizeof(sbv_ota_general_cfg), SBV_OTA_CONFIG_FLASH_ADD);
    if (ret != SBV_OK)
    {
        /* LOG */
        return -1;
    }

    SBV_OTA_DB_MUTEX_UNLOCK;

    return 0;
}

/*
 *@brief: Task to control the actual firmware update process, including 3 main sub-tasks
 *          - Check the metadata of the frimware going to receive
 *          - Receive and write the new firmware to the reserved slot
 *          - Perform firmware signature validation after receive all of the image
 *        Then, reset the system if the new firmware is valid, to let the bootloader boot the new image
 */
void
sbv_ota_update_fw_thread (void *param)
{
    int ret, is_update_enable, read_size, current_page_addr, is_image_valid;
    uint8_t inactive_slot;
    uint32_t slot_pag_add, fw_img_crc;
    sbv_rtos_event_bits_t rcv_data_bits;
    sbv_ota_fw_metadata_t current_fw_metadata;
    sbv_rtos_tick_type_t tick_to_wait = sbv_rtos_ms_to_tick(portMAX_DELAY);

    current_page_addr   = 0;
    is_update_enable    = SBV_TRUE;
    for(;;)
    {
        rcv_data_bits = sbv_rtos_event_group_wait_bits(sbv_ota_event_group,
                                                       SBV_OTA_RCV_METADATA | SBV_OTA_RCV_PAGE | SBV_OTA_RCV_ALL,
                                                       SBV_RTOS_TRUE, SBV_RTOS_FALSE,
                                                       tick_to_wait);
        if(rcv_data_bits & SBV_OTA_RCV_METADATA)
        {
            sbv_rtos_mutex_lock(sbv_ota_msg_rx_instance.mutex);

            ret = sbv_cqbuff_read (sbv_ota_msg_rx_instance.rx_queue, (unsigned char *)&fw_metadata, sizeof (sbv_ota_fw_metadata_t));
            if (ret != sizeof (sbv_ota_fw_metadata_t))
            {
                /* LOG */
                is_update_enable = SBV_FALSE;
                sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);
                continue;
            }

            sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);

            ret = sbv_ota_get_current_fw_metadata (&current_fw_metadata);
            if (ret == 0)
            {
                /* LOG */
                is_update_enable = SBV_FALSE;
                continue;
            }

            /* Check if the on going fw has the same metadata or not, to continue the process */
            if (! memcmp (&fw_metadata, &current_fw_metadata, sizeof(sbv_ota_fw_metadata_t)))
            {
                /* LOG */
                is_update_enable = SBV_FALSE;
            }
            else
            {
                /* Erase the FLASH memory of the inactive slot */
                inactive_slot = sbv_ota_get_available_slot_num ();
                if (inactive_slot != SBV_OTA_INVALID_SLOT)
                {
                    slot_pag_add = SBV_OTA_SLOT_PAGE_ADDR(inactive_slot);
                    ret = sbv_ota_erase_flash_pages (slot_pag_add, SBV_OTA_FW_SLOT_PAGES);
                    if (ret != SBV_OK)
                    {
                        /* LOG */
                        is_update_enable = SBV_FALSE;
                    }
                }
            }
        }
        else if(rcv_data_bits & SBV_OTA_RCV_PAGE)
        {
            /* Write pages of data to the FLASH memory */
            read_size = 0;
            if (is_update_enable)
            {
                sbv_rtos_mutex_lock(sbv_ota_msg_rx_instance.mutex);

                while (read_size < SBV_OTA_PAGES_SIZE)
                {
                    ret = sbv_cqbuff_read (sbv_ota_msg_rx_instance.rx_queue, fw_pages, SBV_OTA_PAGES_SIZE);
                    if (ret == 0)
                    {
                        /* LOG */
                        is_update_enable = SBV_FALSE;
                        break;
                    }

                    read_size += ret;
                }

                sbv_rtos_mutex_unlock(sbv_ota_msg_rx_instance.mutex);

                if (read_size == SBV_OTA_PAGES_SIZE)
                {
                    ret = sbv_ota_write_flash_data((uint8_t *)fw_pages, SBV_OTA_PAGES_SIZE,
                                                    slot_pag_add + current_page_addr);
                    if (ret != SBV_OK)
                    {
                        /* LOG */
                        continue;
                    }

                    current_page_addr += SBV_OTA_PAGES_SIZE;
                }
            }
        }
        else if(rcv_data_bits & SBV_OTA_RCV_ALL)
        {
            if (! is_update_enable)
                continue;

            fw_img_crc = sbv_ota_fw_crc_cal (slot_pag_add, fw_metadata);
            is_image_valid = (fw_img_crc == fw_metadata.fw_crc) ? SBV_TRUE : SBV_FALSE;

            /* Send report msg */


            /* Save metadata of system into Flash */
            ret = sbv_ota_save_fw_img_cfg (inactive_slot, &fw_metadata, is_image_valid);
            if (ret != 0)
            {
                /* LOG */
                continue;
            }

            /* Reset the system after 10s */
            sbv_rtos_task_delay(sbv_rtos_ms_to_tick(SBV_OTA_LOAD_NEW_FW_APP_WAIT_MS));
#ifdef STM32F1xx
            HAL_NVIC_SystemReset ();
#endif /* STM32F1xx */
        }
    }
}

static void
sbv_ota_goto_application (uint32_t slot_addr)
{
    /* Set the function pointer to the start of the app memory address */
    void (*AppReset_Handler)(void) = (void*)(*((volatile uint32_t*)(slot_addr + 4U)));
	if(AppReset_Handler == (void*)0xFFFFFFFF)
		return;

#ifdef STM32F1xx
    /* Disable all interrupts */
    __disable_irq();

    /* Reset the Clock */
    HAL_RCC_DeInit();
    HAL_DeInit();

    /* Clear all pending interrupts */
    NVIC_ClearPendingIRQ((IRQn_Type)0);

    /* Set new Vector Table Offset */
    SCB->VTOR = slot_addr;

    /* Set the main stack pointer to the application slot */
    __set_MSP(*(volatile uint32_t*) slot_addr);

    /* Disable Systick interrupt */
    SysTick->CTRL   = 0;
    SysTick->LOAD   = 0;
    SysTick->VAL    = 0;
#endif /*STM32F1xx*/

	AppReset_Handler();
}

void
sbv_ota_load_new_app (void)
{
    uint8_t fw_slot, i;
    uint32_t slot_addr;

    /* Built-in LED initialization */
    sbv_gpio_init(SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, SBV_GPIO_MODE_OUTPUT);

    fw_slot = sbv_ota_get_available_fw_update_slot();
    if(fw_slot == SBV_OTA_INVALID_SLOT)
    {
        /* LOG */
        /* Blink the LED each 100ms to indicate that both of the FW slot can not be used */
        while (1)
        {
            sbv_gpio_toggle_pin (SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, 0);
#ifdef STM32F1xx
            HAL_Delay (200);
#endif /* STM32F1xx */
        }
    }

    /* Blink the LED to indicate that it is going to load new FW */
    for (i = 0; i < 20; ++i)
    {
        sbv_gpio_toggle_pin (SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, 0);
#ifdef STM32F1xx
        HAL_Delay (100);
#endif /* STM32F1xx */
    }

    slot_addr = SBV_OTA_SLOT_PAGE_ADDR(fw_slot);
    /* Goto application */
    sbv_ota_goto_application(slot_addr);
}