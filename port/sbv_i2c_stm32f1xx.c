#include <stdlib.h>
#include <string.h>
#include "sbv_rtos.h"
#include "sbv_i2c.h"
#include "sbv_i2c_stm32f1xx.h"

#ifdef STM32F1xx
sbv_i2c_instance_t sbv_i2c_instance;

extern sbv_rtos_mutex_t SBV_I2C_TX_BUFFER_MUTEX;
extern sbv_rtos_mutex_t SBV_I2C_RX_BUFFER_MUTEX;

#define sbv_i2c_stm32f1xx_rx_hw_callback \
        HAL_I2C_MasterRxCpltCallback

void
sbv_i2c_stm32f1xx_master_init(sbv_i2c_handle_t *i2c_handle)
{
    if(! i2c_handle)
        return;

    sbv_i2c_instance.i2c_active            = SBV_TRUE;
    sbv_i2c_instance.i2c_rx_buffer         = (uint8_t *)sbv_rtos_malloc(sizeof(uint8_t) * SBV_I2C_RX_BUFFER_SIZE);
    if (! sbv_i2c_instance.i2c_rx_buffer)
    {
        /* LOG */
        goto ERR_EXIT;
    }
    sbv_i2c_instance.i2c_tx_buffer         = (uint8_t *)sbv_rtos_malloc(sizeof(uint8_t) * SBV_I2C_TX_BUFFER_SIZE);
    if (! sbv_i2c_instance.i2c_rx_buffer)
    {
        /* LOG */
        goto ERR_EXIT;
    }
    sbv_i2c_instance.i2c_rx_buffer_pos     = 0;
    sbv_i2c_instance.i2c_rx_isr_size       = 0;
    sbv_i2c_instance.i2c_handle            = i2c_handle;
    sbv_i2c_instance.i2c_rx_notify_task    = NULL;

    /* Create the mutex for the I2C TX and RX FIFO */
    sbv_rtos_mutex_create(SBV_I2C_TX_BUFFER_MUTEX);
    sbv_rtos_mutex_create(SBV_I2C_RX_BUFFER_MUTEX);

    /* Initiate the tx and rx buffers */
    memset(sbv_i2c_instance.i2c_rx_buffer, 0, SBV_I2C_TX_BUFFER_SIZE);
    memset(sbv_i2c_instance.i2c_tx_buffer, 0, SBV_I2C_RX_BUFFER_SIZE);

    return;
ERR_EXIT:
    if (sbv_i2c_instance.i2c_rx_buffer)    sbv_rtos_free (sbv_i2c_instance.i2c_rx_buffer);
    return;
}

static int
sbv_i2c_stm32f1xx_tx_packet_format(uint8_t* i2c_tx_data, uint16_t i2c_tx_size, uint8_t* i2c_tx_buffer)
{
    int sent_bytes;

    if(!i2c_tx_data || !i2c_tx_buffer)
        return 0;

    memset(i2c_tx_buffer, 0, SBV_I2C_TX_BUFFER_SIZE);

    if(i2c_tx_size <= SBV_I2C_TX_BUFFER_SIZE)
    {
        memcpy(i2c_tx_buffer, i2c_tx_data, i2c_tx_size);
        sent_bytes = i2c_tx_size;
    }
    else
    {
        memcpy(i2c_tx_buffer, i2c_tx_data, SBV_I2C_TX_BUFFER_SIZE);
        sent_bytes = SBV_I2C_TX_BUFFER_SIZE;
    }

    return sent_bytes;
}

static int
sbv_i2c_stm32f1xx_master_tx_send_pkt (sbv_i2c_handle_t* i2c_handle, uint8_t slave_add,
                                      sbv_i2c_msg_t msg_type, uint8_t* i2c_tx_buffer, uint16_t i2c_tx_size)
{
    if(!i2c_handle || !i2c_tx_buffer)
        return SBV_ERROR;

    return HAL_I2C_Master_Transmit (i2c_handle, ((slave_add << 0x1) | msg_type), i2c_tx_buffer,
                                    i2c_tx_size, SBV_I2C_TX_TIMEOUT);
}

int
sbv_i2c_stm32f1xx_master_send_data (uint8_t slave_add, sbv_i2c_msg_t msg_type,
                                    uint8_t* i2c_tx_data, uint16_t i2c_tx_size)
{
    int ret = SBV_OK, total_tx_bytes = 0, cur_tx_bytes = 0;

    if(! i2c_tx_data)
        return SBV_ERROR;

    SBV_I2C_TX_BUFFER_MUTEX_LOCK;

    while (total_tx_bytes < i2c_tx_size)
    {
        cur_tx_bytes = sbv_i2c_stm32f1xx_tx_packet_format (i2c_tx_data + total_tx_bytes,
                                                          (i2c_tx_size - total_tx_bytes),
                                                          sbv_i2c_instance.i2c_tx_buffer);
        ret = sbv_i2c_stm32f1xx_master_tx_send_pkt (sbv_i2c_instance.i2c_handle, slave_add, msg_type,
                                                    sbv_i2c_instance.i2c_tx_buffer, cur_tx_bytes);
        if (ret != SBV_OK)
            continue;
        total_tx_bytes += cur_tx_bytes;
    }

    SBV_I2C_TX_BUFFER_MUTEX_UNLOCK;

    return total_tx_bytes;
}

void
sbv_i2c_stm32f1xx_rx_hw_callback(sbv_i2c_handle_t* i2c_handle)
{
    sbv_rtos_base_type_t xHigherPriorityTaskWoken = SBV_RTOS_FALSE;

    if(! i2c_handle)
        return;

    if(sbv_i2c_instance.i2c_rx_notify_task != NULL)
    {
        sbv_rtos_notify_give_fromISR(sbv_i2c_instance.i2c_rx_notify_task, &xHigherPriorityTaskWoken);
        sbv_rtos_port_yield_fromISR(xHigherPriorityTaskWoken);
    }
}

uint8_t *
sbv_i2c_stm32f1xx_master_rcv_data (uint8_t slave_add, uint16_t *size)
{
    sbv_rtos_tick_type_t tick_to_wait;

    tick_to_wait = SBV_I2C_RX_TIMEOUT;

    SBV_I2C_RX_BUFFER_MUTEX_LOCK;

    sbv_i2c_instance.i2c_rx_notify_task = sbv_rtos_get_current_task_handle();

    /* Re-initiate the I2C DMA master RX interrutp */
    HAL_I2C_Master_Receive_DMA  (sbv_i2c_instance.i2c_handle,
                                ((slave_add << 0x01) | SBV_I2C_MSG_READ),
                                sbv_i2c_instance.i2c_rx_buffer, SBV_I2C_RX_BUFFER_SIZE);

    sbv_rtos_notify_take(SBV_RTOS_TRUE, tick_to_wait);
    sbv_i2c_instance.i2c_rx_notify_task = NULL;

    SBV_I2C_RX_BUFFER_MUTEX_UNLOCK;

    *size = SBV_I2C_RX_BUFFER_SIZE;
    return sbv_i2c_instance.i2c_rx_buffer;
}
#endif /*STM32F1xx*/