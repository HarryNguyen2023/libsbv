#include <stdlib.h>
#include <string.h>
#include "sbv_rtos.h"
#include "sbv_i2c.h"
#include "sbv_i2c_stm32f1xx.h"

#ifdef STM32F1xx
struct sbv_i2c_instance_list_t sbv_i2c_instance_list = {
    .list = {NULL, NULL}
};

#define SBV_I2C_RX_BUFFER_MUTEX_LOCK(Instance) \
        sbv_rtos_mutex_lock(Instance->mutex)

#define SBV_I2C_RX_BUFFER_MUTEX_UNLOCK(Instance) \
        sbv_rtos_mutex_unlock(Instance->mutex)

#define sbv_i2c_stm32f1xx_rx_hw_callback \
        HAL_I2C_MasterRxCpltCallback

/* Return the I2C instance that owns the given HAL I2C handle. */
static sbv_i2c_instance_t*
sbv_i2c_stm32f1xx_master_get_instance_by_handle (sbv_i2c_handle_t* i2c_handle)
{
    if (! i2c_handle)
        return NULL;

    for (uint8_t i = 0; i < SBV_I2C_MAX_CHANNEL; ++i)
    {
        if (sbv_i2c_instance_list.list[i]
            && sbv_i2c_instance_list.list[i]->i2c_handle == i2c_handle)
        {
           return sbv_i2c_instance_list.list[i];
        }
    }

    return NULL;
}

/* Register a new I2C instance in the internal instance list. */
static int
sbv_i2c_stm32f1xx_master_add_instance_to_list (sbv_i2c_instance_t *i2c_instance)
{
    if (! i2c_instance)
        return SBV_ERROR;

    for (uint8_t i = 0; i < SBV_I2C_MAX_CHANNEL; ++i)
    {
        if (sbv_i2c_instance_list.list[i] == NULL)
        {
           sbv_i2c_instance_list.list[i] =  i2c_instance;
           return SBV_OK;
        }
    }

    return SBV_ERROR;
}

/* Initialize an I2C instance, attach its HAL handle, and prepare the RX buffer state. */
int
sbv_i2c_stm32f1xx_master_init(sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle)
{
    if(! i2c_instance || ! i2c_handle)
        return SBV_ERROR;

    if (sbv_i2c_stm32f1xx_master_add_instance_to_list(i2c_instance) != SBV_OK)
    {
        // LOG
        return SBV_ERROR;
    }

    i2c_instance->i2c_rx_buffer_pos     = 0;
    i2c_instance->i2c_handle            = i2c_handle;
    i2c_instance->i2c_rx_notify_task    = NULL;

    /* Create the mutex for the I2C RX FIFO */
    sbv_rtos_mutex_create(i2c_instance->mutex);

    /* Initiate the tx and rx buffers */
    memset(i2c_instance->i2c_rx_buffer, 0, SBV_I2C_RX_BUFFER_SIZE);

    return SBV_OK;
}

/* Transmit a single I2C packet to the target slave using the HAL master transmit API. */
static int
sbv_i2c_stm32f1xx_master_tx_send_pkt (sbv_i2c_handle_t* i2c_handle, uint8_t slave_add,
                                      sbv_i2c_msg_t msg_type, uint8_t* i2c_tx_data, uint16_t i2c_tx_size)
{
    if(! i2c_handle || ! i2c_tx_data)
        return SBV_ERROR;

    return HAL_I2C_Master_Transmit (i2c_handle, ((slave_add << 0x1) | msg_type),
                                    i2c_tx_data, i2c_tx_size, SBV_I2C_TX_TIMEOUT);
}

/* Send data from a configured I2C instance to a slave device. */
int
sbv_i2c_stm32f1xx_master_send_data (sbv_i2c_instance_t *i2c_instance,
                                    uint8_t slave_add, sbv_i2c_msg_t msg_type,
                                    uint8_t* i2c_tx_data, uint16_t i2c_tx_size)
{
    int ret = SBV_OK;

    if(! i2c_instance || ! i2c_tx_data)
        return SBV_ERROR;

    ret = sbv_i2c_stm32f1xx_master_tx_send_pkt (i2c_instance->i2c_handle, slave_add,
                                                msg_type, i2c_tx_data, i2c_tx_size);
    if (ret != SBV_OK)
        return SBV_ERROR;

    return ret;
}

/* Handle the hardware RX-complete interrupt and notify the waiting task. */
void
sbv_i2c_stm32f1xx_rx_hw_callback(sbv_i2c_handle_t* i2c_handle)
{
    sbv_i2c_instance_t *i2c_instance = NULL;
    sbv_rtos_base_type_t xHigherPriorityTaskWoken = SBV_RTOS_FALSE;

    if(! i2c_handle)
        return;

    i2c_instance = sbv_i2c_stm32f1xx_master_get_instance_by_handle (i2c_handle);
    if (i2c_instance == NULL)
    {
        // LOG
        return;
    }

    if(i2c_instance->i2c_rx_notify_task != NULL)
    {
        sbv_rtos_notify_give_fromISR(i2c_instance->i2c_rx_notify_task, &xHigherPriorityTaskWoken);
        sbv_rtos_port_yield_fromISR(xHigherPriorityTaskWoken);
    }
}

/* Receive data from a slave using DMA and copy it into the caller buffer. */
int
sbv_i2c_stm32f1xx_master_rcv_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                                   uint8_t received_buf[], uint16_t size)
{
    sbv_rtos_tick_type_t tick_to_wait;
    int recv_size = 0, ret = SBV_OK;

    if (! i2c_instance || ! received_buf)
        return SBV_ERROR;

    tick_to_wait = SBV_I2C_RX_TIMEOUT;

    SBV_I2C_RX_BUFFER_MUTEX_LOCK(i2c_instance);

    i2c_instance->i2c_rx_notify_task = sbv_rtos_get_current_task_handle();

    recv_size = (size < SBV_I2C_RX_BUFFER_SIZE) ? size : SBV_I2C_RX_BUFFER_SIZE;

    /* Re-initiate the I2C DMA master RX interrutp */
    ret = HAL_I2C_Master_Receive_DMA(i2c_instance->i2c_handle,
                                    ((slave_add << 0x01) | SBV_I2C_MSG_READ),
                                    i2c_instance->i2c_rx_buffer, recv_size);
    if (ret != SBV_OK)
    {
        // LOG
        SBV_I2C_RX_BUFFER_MUTEX_UNLOCK(i2c_instance);
        return SBV_ERROR;
    }

    sbv_rtos_notify_take(SBV_RTOS_TRUE, tick_to_wait);
    i2c_instance->i2c_rx_notify_task = NULL;

    memcpy (received_buf, i2c_instance->i2c_rx_buffer, recv_size);

    SBV_I2C_RX_BUFFER_MUTEX_UNLOCK(i2c_instance);

    return recv_size;
}
#endif /*STM32F1xx*/