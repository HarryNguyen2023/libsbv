#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_gpio.h"
#include "sbv_cqbuff.h"
#include "sbv_ota_common.h"
#include "sbv_ota.h"
#include "sbv_ota_msg.h"

#define SBV_OTA_LOAD_NEW_FW_APP_WAIT_MS (5 * 1000)
#define SBV_OTA_UPDATE_FW_PRIO          2
#define SBV_OTA_UPDATE_WATCHDOG_MS      (5 * 1000)
#define SBV_OTA_MSG_QUEUE_TX_TIMEOUT_MS (100)

#define SBV_OTA_QUEUE_LEN   1

sbv_rtos_task_handle_t sbv_ota_update_fw_handle;
static sbv_rtos_stack_type_t sbv_ota_fw_update_stack[STACK_SIZE_BASE * 4];

void sbv_ota_update_fw_thread (void *param);

sbv_ota_installer_t sbv_ota_installer;

void
sbv_ota_ipc_queue_init (sbv_ota_ipc_t *ipc) {
    if (! ipc) {
        // LOG
        return;
    }

    ipc->to_installer = sbv_rtos_create_queue (SBV_OTA_QUEUE_LEN, sizeof (sbv_ota_system_msg_t));
    ipc->to_slave_fsm = sbv_rtos_create_queue (SBV_OTA_QUEUE_LEN, sizeof (sbv_ota_system_msg_t));
}

void
sbv_ota_update_init(void *param)
{
    sbv_ota_ipc_t *ipc;

    memset (&sbv_ota_installer, 0, sizeof (sbv_ota_installer_t));
    sbv_ota_installer.is_update_enable          = SBV_TRUE;
    sbv_ota_installer.is_updating               = SBV_FALSE;
    sbv_ota_installer.current_flash_page_addr   = 0;

    sbv_ota_installer.inactive_slot = SBV_OTA_INVALID_SLOT;
    sbv_ota_installer.slot_pag_add  = 0;

    ipc = (sbv_ota_ipc_t *)param;
    if (! ipc) {
        // LOG
        return;
    }
    sbv_ota_installer.rx_queue = ipc->to_installer;
    sbv_ota_installer.tx_queue = ipc->to_slave_fsm;

    sbv_rtos_mutex_create (sbv_ota_installer.mutex);

    sbv_rtos_task_create(sbv_ota_update_fw_thread, "update_fw", STACK_SIZE_BASE * 4,
                         NULL, SBV_OTA_UPDATE_FW_PRIO, sbv_ota_fw_update_stack, &sbv_ota_update_fw_handle);
}

uint8_t
sbv_ota_is_update_enable (void)
{
    return sbv_ota_installer.is_update_enable;
}

void
sbv_ota_set_update_enable (uint8_t is_update_enable)
{
    sbv_ota_installer.is_update_enable = is_update_enable;
}

uint8_t
sbv_ota_is_updating (void)
{
    return sbv_ota_installer.is_updating;
}

uint8_t
sbv_ota_is_updating_locked (void)
{
    uint8_t updating;

    sbv_rtos_mutex_lock(sbv_ota_installer.mutex);

    updating = sbv_ota_is_updating();

    sbv_rtos_mutex_unlock(sbv_ota_installer.mutex);

    return updating;
}

void
sbv_ota_set_update_status (uint8_t is_updating)
{
    sbv_ota_installer.is_updating = is_updating;
}


int
sbv_ota_send_system_msg_ack (sbv_rtos_queue_handle_t queue, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_ACK, NULL, timeout_ms);
}

int
sbv_ota_send_system_msg_abort (sbv_rtos_queue_handle_t queue, uint16_t timeout_ms) {
    return sbv_ota_send_system_msg (queue, SBV_OTA_EVENT_ABORT, NULL, timeout_ms);
}

void
sbv_ota_abort_fw_update (void)
{
    sbv_rtos_mutex_lock(sbv_ota_installer.mutex);

    // Reset the state of the rx instance
    sbv_ota_installer.is_update_enable          = SBV_TRUE;
    sbv_ota_installer.is_updating               = SBV_FALSE;
    sbv_ota_installer.current_flash_page_addr   = 0;

    memset (&(sbv_ota_installer.fw_metadata), 0, sizeof (sbv_ota_fw_metadata_t));
    sbv_ota_installer.fw_data_addr  = NULL;

    sbv_ota_installer.inactive_slot = SBV_OTA_INVALID_SLOT;
    sbv_ota_installer.slot_pag_add  = 0;

    sbv_ota_send_system_msg_abort(sbv_ota_installer.tx_queue, SBV_OTA_MSG_QUEUE_TX_TIMEOUT_MS);

    sbv_rtos_mutex_unlock(sbv_ota_installer.mutex);
}

int
sbv_ota_save_fw_img_cfg (uint16_t image_slot, sbv_ota_fw_metadata_t* slot_metadata, int is_image_valid)
{
    int ret;
	sbv_ota_general_cfg_t cfg;

    if (! sbv_ota_is_valid_fw_slot (image_slot) || slot_metadata == NULL)
    {
        // LOG
        return -1;
    }

    /* Read the configuration in flash memory space */
    ret = sbv_ota_cfg_read_and_validate (&cfg);
    if (ret != SBV_OK) {
        // LOG
        return -1;
    }

    if (is_image_valid)
    {
        cfg.reboot_reason                         = SBV_OTA_NEW_UPDATE_BOOT;
        cfg.slot_table[image_slot].is_slot_active = SBV_FALSE;
        cfg.slot_table[image_slot].is_slot_valid  = SBV_TRUE;
        cfg.slot_table[image_slot].is_slot_update = SBV_TRUE;
        memcpy (&(cfg.slot_table[image_slot].metadata), slot_metadata, sizeof (sbv_ota_fw_metadata_t));
    }
    else
    {
        cfg.reboot_reason                         = SBV_OTA_POWER_UP_BOOT;
        cfg.slot_table[image_slot].is_slot_active = SBV_FALSE;
        cfg.slot_table[image_slot].is_slot_valid  = SBV_FALSE;
        cfg.slot_table[image_slot].is_slot_update = SBV_FALSE;
        memcpy (&(cfg.slot_table[image_slot].metadata), slot_metadata, sizeof (sbv_ota_fw_metadata_t));
    }

    ret = sbv_ota_cfg_commit (&cfg);
    if (ret != SBV_OK)
    {
        /* LOG */
        return -1;
    }

    return 0;
}

int
sbv_ota_fw_metadata_validate (sbv_ota_fw_metadata_t *fw_metadata,
                              uint32_t* slot_pag_add, uint8_t* inactive_slot)
{
    int ret, cmp_version, cmp_timestampt;
    sbv_ota_fw_metadata_t current_fw_metadata;

    if (! fw_metadata || ! slot_pag_add || ! inactive_slot)
        return SBV_ERROR;

    if (! sbv_ota_is_updating ()) {
        // LOG
        return SBV_ERROR;
    }

    // Check if the new firmware size is larger the allowable size of A/B partition
    // to avoid the firmware overload attack
    if (fw_metadata->fw_size == 0 || fw_metadata->fw_size > SBV_OTA_SLOT_MAX_SIZE)
    {
        /* LOG */
        return SBV_ERROR;
    }

    ret = sbv_ota_get_current_fw_metadata (&current_fw_metadata);
    if (ret != 0)
    {
        /* LOG */
        return SBV_ERROR;
    }

    // Compare firmware timestampt to avoid old firmware or future enforcement attack
    cmp_timestampt = memcmp (&(fw_metadata->fw_timestamp), &(current_fw_metadata.fw_timestamp), SBV_OTA_FW_TIME_LENGTH);
    if (cmp_timestampt < 0) {
        // LOG
        return SBV_ERROR;
    }
    // TODO: check the new fw timestampt with current system time

    /* Check if the on going fw has the same metadata or not, to continue the process */
    cmp_version = memcmp (&(fw_metadata->fw_version), &(current_fw_metadata.fw_version), SBV_OTA_FW_VERSION_LENGTH);
    if (cmp_version == 0)
    {
        /* LOG */
        return SBV_ERROR;
    }
    // Avoid firmware rollback attack
    else if (cmp_version < 0)
    {
        /* LOG */
        return SBV_ERROR;
    }
    else
    {
        /* Erase the FLASH memory of the inactive slot */
        *inactive_slot = sbv_ota_get_available_slot_num ();
        if (*inactive_slot == SBV_OTA_INVALID_SLOT)
        {
            /* LOG */
            return SBV_ERROR;
        }

        *slot_pag_add = SBV_OTA_SLOT_PAGE_ADDR(*inactive_slot);
        ret = sbv_ota_erase_flash_data (*slot_pag_add, SBV_OTA_FW_SLOT_PAGES);
        if (ret != SBV_OK)
        {
            /* LOG */
            return SBV_ERROR;
        }
    }

    sbv_ota_send_system_msg_ack(sbv_ota_installer.tx_queue, SBV_OTA_MSG_QUEUE_TX_TIMEOUT_MS);

    return SBV_OK;
}

int
sbv_ota_save_fw_image_to_flash (uint8_t *fw_img, const uint32_t fw_size, const uint32_t slot_pag_addr)
{
    int ret;
    uint8_t *curr_fw_img_head;
    uint32_t page_num;
    uint32_t last_page_size;
    uint32_t curr_page_addr;

    if (! fw_img || fw_size == 0 || fw_size > SBV_OTA_SLOT_MAX_SIZE) {
        // LOG
        return SBV_ERROR;
    }

    if (! sbv_ota_is_updating ()) {
        // LOG
        return SBV_ERROR;
    }

    if (! sbv_ota_is_valid_page_addr (slot_pag_addr)) {
        // LOG
        return SBV_ERROR;
    }

    curr_page_addr      = slot_pag_addr;
    curr_fw_img_head    = fw_img;

    last_page_size = (fw_size % SBV_OTA_PAGES_SIZE);
    page_num = (last_page_size == 0) ? \
                    (fw_size / SBV_OTA_PAGES_SIZE) : (fw_size / SBV_OTA_PAGES_SIZE + 1);

    for (uint32_t i = 0; i < page_num; ++i) {
        if (last_page_size && i == (page_num - 1)) {
            ret = sbv_ota_write_flash_data (curr_fw_img_head, last_page_size, curr_page_addr);
            if (ret != SBV_OK) {
                // LOG
                return ret;
            }
        } else {
            ret = sbv_ota_write_flash_data (curr_fw_img_head, SBV_OTA_PAGES_SIZE, curr_page_addr);
            if (ret != SBV_OK) {
                // LOG
                return ret;
            }

            curr_page_addr      += SBV_OTA_PAGES_SIZE;
            curr_fw_img_head    += SBV_OTA_PAGES_SIZE;
        }
    }

    sbv_ota_send_system_msg_ack(sbv_ota_installer.tx_queue, SBV_OTA_MSG_QUEUE_TX_TIMEOUT_MS);

    return SBV_OK;
}

void
sbv_ota_system_reset (void)
{
    sbv_ota_set_update_enable (SBV_FALSE);

    /* Reset the system after 10s */
    sbv_rtos_task_delay(sbv_rtos_ms_to_tick(SBV_OTA_LOAD_NEW_FW_APP_WAIT_MS));

#ifdef STM32F1xx
    HAL_NVIC_SystemReset ();
#endif /* STM32F1xx */
}

int
sbv_ota_handle_final_upd (const uint32_t slot_pag_addr, const uint8_t inactive_slot,
                          const sbv_ota_fw_metadata_t fw_metadata)
{
    int ret, is_image_valid;

    if (! sbv_ota_is_updating ()) {
        // LOG
        return SBV_ERROR;
    }

    if (! sbv_ota_is_valid_page_addr (slot_pag_addr))
    {
        // LOG
        return SBV_ERROR;
    }

    is_image_valid = sbv_ota_fw_image_crc_validate (slot_pag_addr, fw_metadata);
    if (! is_image_valid)
    {
        // LOG
        return SBV_ERROR;
    }

    /* Save metadata of system into Flash */
    ret = sbv_ota_save_fw_img_cfg (inactive_slot, &fw_metadata, is_image_valid);
    if (ret != 0)
    {
        /* LOG */
        return SBV_ERROR;
    }

    sbv_ota_send_system_msg_ack(sbv_ota_installer.tx_queue, SBV_OTA_MSG_QUEUE_TX_TIMEOUT_MS);

    sbv_ota_system_reset ();

    return SBV_OK;
}

int
sbv_ota_process_update_fw_cmd (void)
{
    int ret;
    uint8_t inactive_slot;
    uint32_t slot_pag_add;
    uint8_t *fw_data;
    sbv_ota_fw_metadata_t *fw_metadata = NULL;
    sbv_ota_system_msg_t *rcv_msg;
    
    sbv_rtos_mutex_lock(sbv_ota_installer.mutex);

    rcv_msg = (sbv_ota_system_msg_t *)(sbv_ota_installer.data);
    if (rcv_msg == NULL)
    {
        // LOG
        goto ERR_EXIT;
    }

    if (! sbv_ota_is_update_enable())
    {
        // LOG
        goto ERR_EXIT;
    }

    switch (rcv_msg->event)
    {
    case SBV_OTA_EVENT_UDP_START:
        slot_pag_add  = 0;
        inactive_slot = SBV_OTA_INVALID_SLOT;
        sbv_ota_set_update_status (SBV_TRUE);

        fw_metadata = (sbv_ota_fw_metadata_t *)rcv_msg->data;
        memcpy (&(sbv_ota_installer.fw_metadata), fw_metadata, sizeof(sbv_ota_fw_metadata_t));

        ret = sbv_ota_fw_metadata_validate (&(sbv_ota_installer.fw_metadata), &slot_pag_add, &inactive_slot);
        if (ret != SBV_OK)
        {
            /* LOG */
            goto ERR_EXIT;
        }

        sbv_ota_installer.inactive_slot = inactive_slot;
        sbv_ota_installer.slot_pag_add  = slot_pag_add;
        break;

    case SBV_OTA_EVENT_IMG_WRITE:
        fw_data = (uint8_t *)rcv_msg->data;
        ret = sbv_ota_save_fw_image_to_flash (fw_data, sbv_ota_installer.fw_metadata.fw_size,
                                              sbv_ota_installer.slot_pag_add);
        if (ret != SBV_OK)
        {
            // LOG
            goto ERR_EXIT;
        }
        break;

    case SBV_OTA_EVENT_UDP_FINALIZE:
        ret = sbv_ota_handle_final_upd (sbv_ota_installer.slot_pag_add,
                                        sbv_ota_installer.inactive_slot,
                                        sbv_ota_installer.fw_metadata);
        if (ret != SBV_OK)
        {
            // LOG
            goto ERR_EXIT;
        }
        break;

    default:
        goto ERR_EXIT;
    }

    sbv_rtos_mutex_unlock(sbv_ota_installer.mutex);
    return SBV_OK;

ERR_EXIT:
    sbv_rtos_mutex_unlock(sbv_ota_installer.mutex);
    return -1;
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
    int ret;

    sbv_rtos_base_type_t status;
    sbv_rtos_tick_type_t tick_to_wait;

    for(;;)
    {
        // To protect against partial firmware update attack, we use a simple
        // watchdog timer mechanism to abort the upate process if the next event
        // does not arrive in time.
        tick_to_wait = sbv_ota_is_updating_locked() ? sbv_rtos_ms_to_tick(SBV_OTA_UPDATE_WATCHDOG_MS) : portMAX_DELAY;

        status = sbv_rtos_queue_rcv(sbv_ota_installer.rx_queue, sbv_ota_installer.data, tick_to_wait);
        if (status != SBV_RTOS_TRUE)
        {
            if (sbv_ota_is_updating_locked())
            {
                /* Watchdog timeout: no next OTA event arrived in time */
                sbv_ota_abort_fw_update();
            }
        }
        else
        {
            ret = sbv_ota_process_update_fw_cmd();
            if (ret != SBV_OK)
            {
                // LOG
                sbv_ota_abort_fw_update();
            }
        }
    }
}

