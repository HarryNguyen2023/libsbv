#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_gpio.h"
#include "sbv_ota_common.h"
#include "sbv_ota_bootloader.h"

/*
 * @brief: Get the available data slot for firmware update
 * @param none
 * @retval uint8_t
 */
static uint8_t
sbv_ota_get_available_fw_update_slot (void)
{
    int ret;
	uint8_t data_slot = SBV_OTA_INVALID_SLOT;
    uint8_t is_image_valid = SBV_FALSE, is_update_cfg = SBV_FALSE;
    uint32_t slot_page_addr;
    sbv_ota_general_cfg_t cfg;

	/* Read the configuration in flash memory space */
	ret = sbv_ota_cfg_read_and_validate (&cfg);
    if (ret != SBV_OK) {
        // LOG
        return SBV_OTA_INVALID_SLOT;
    }

	/* Looking for the new update slot and boot it up */
    if (cfg.reboot_reason == SBV_OTA_NEW_UPDATE_BOOT)
    {
        data_slot = sbv_ota_get_update_slot(&cfg);
        if (data_slot == SBV_OTA_INVALID_SLOT)
        {
            // LOG
            return SBV_OTA_INVALID_SLOT;
        }

        /* Perform checking of the new fw image signature */
        slot_page_addr = SBV_OTA_SLOT_PAGE_ADDR (data_slot);
        is_image_valid = sbv_ota_fw_image_crc_validate (slot_page_addr, cfg.slot_table[data_slot].metadata);

        is_update_cfg = SBV_TRUE;
        cfg.reboot_reason = SBV_OTA_POWER_UP_BOOT;
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
            goto FALL_BACK;   // Fall back to boot the other already available partition
        }
    }
    else if (cfg.reboot_reason == SBV_OTA_POWER_UP_BOOT)
    {
FALL_BACK:
        data_slot = sbv_ota_get_active_slot (&cfg);
        if (data_slot == SBV_OTA_INVALID_SLOT)
        {
            // LOG
            return SBV_OTA_INVALID_SLOT;
        }

        /* Perform checking of the other fw image signature */
        slot_page_addr = SBV_OTA_SLOT_PAGE_ADDR (data_slot);
        is_image_valid = sbv_ota_fw_image_crc_validate (slot_page_addr, cfg.slot_table[data_slot].metadata);
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
        ret = sbv_ota_cfg_commit (&cfg);
        if (ret != SBV_OK)
        {
            /* LOG */
            return SBV_OTA_INVALID_SLOT;
        }
    }
    
	return data_slot;
}

static void
sbv_ota_bootloader_reset_system (uint32_t slot_addr)
{
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
}

static void
sbv_ota_bootloader_goto_application (uint32_t slot_addr)
{
    /* Set the function pointer to the start of the app memory address */
    void (*AppReset_Handler)(void) = (void*)(*((volatile uint32_t*)(slot_addr + 4U)));
	if(AppReset_Handler == (void*)0xFFFFFFFF)
		return;

    sbv_ota_bootloader_reset_system (slot_addr);

	AppReset_Handler();
}

static void
sbv_ota_bootloader_init_blink (void)
{
    uint16_t blink_time_ms          = 5000;
    uint16_t blink_toggle_time_ms   = 200;

    for (uint8_t i = 0; i < (blink_time_ms / blink_toggle_time_ms); ++i) {
        sbv_gpio_toggle_pin (SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, 0);
#ifdef STM32F1xx
        HAL_Delay (blink_toggle_time_ms);
#endif /* STM32F1xx */
    }
}


void
sbv_ota_bootloader_load_new_app (void)
{
    uint8_t fw_slot;
    uint32_t slot_addr;

    /* Built-in LED initialization */
    sbv_gpio_init(SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, SBV_GPIO_MODE_OUTPUT);

    fw_slot = sbv_ota_get_available_fw_update_slot();
    if(fw_slot == SBV_OTA_INVALID_SLOT)
    {
        /* LOG */
        /* Blink the LED each 500ms to indicate that both of the FW slot can not be used */
        while (1)
        {
            sbv_gpio_toggle_pin (SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, 0);
#ifdef STM32F1xx
            HAL_Delay (500);
#endif /* STM32F1xx */
        }
    }

    /* Blink the LED to indicate that it is going to load new FW */
    sbv_ota_bootloader_init_blink();

    slot_addr = SBV_OTA_SLOT_PAGE_ADDR(fw_slot);
    /* Goto application */
    sbv_ota_bootloader_goto_application(slot_addr);
}