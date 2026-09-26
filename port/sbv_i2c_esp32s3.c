#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_log.h"
#include "sbv_rtos.h"
#include "sbv_i2c.h"
#include "sbv_i2c_esp32s3.h"

struct sbv_i2c_instance_list_t sbv_i2c_instance_list = {0};

#define SBV_I2C_RX_BUFFER_MUTEX_LOCK(Instance) \
        sbv_rtos_mutex_lock(Instance->mutex)

#define SBV_I2C_RX_BUFFER_MUTEX_UNLOCK(Instance) \
        sbv_rtos_mutex_unlock(Instance->mutex)

/* Return the I2C instance that owns the given HAL I2C handle. */
static sbv_i2c_instance_t*
sbv_i2c_esp32s3_master_get_instance_by_handle (sbv_i2c_handle_t* i2c_handle)
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
sbv_i2c_esp32s3_master_add_instance_to_list (sbv_i2c_instance_t *i2c_instance)
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
sbv_i2c_esp32s3_master_init(sbv_i2c_instance_t *i2c_instance,
                            sbv_i2c_handle_t *i2c_handle,
                            uint8_t slave_addr)
{
    if(! i2c_instance || ! i2c_handle)
        return SBV_ERROR;

    if (sbv_i2c_esp32s3_master_add_instance_to_list(i2c_instance) != SBV_OK)
    {
        LOG_ERROR ("Failed to add new I2C instance to list");
        return SBV_ERROR;
    }

    /* Create the mutex for the I2C RX FIFO */
    sbv_rtos_mutex_create(i2c_instance->mutex);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port   = *i2c_handle,
        .sda_io_num = GPIO_NUM_9,
        .scl_io_num = GPIO_NUM_8,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_cfg, &bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length  = I2C_ADDR_BIT_LEN_7,
        .device_address   = slave_addr,
        .scl_speed_hz     = 400000,
    };
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);

    i2c_instance->i2c_handle = dev_handle;

    return SBV_OK;
}

/* Send data from a configured I2C instance to a slave device. */
int
sbv_i2c_esp32s3_master_send_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                                  sbv_i2c_msg_t msg_type, uint8_t* i2c_tx_data,
                                  uint16_t i2c_tx_size, uint16_t timeout_ms)
{
    int ret = SBV_OK;

    if(! i2c_instance || ! i2c_tx_data)
        return SBV_ERROR;

    ret = i2c_master_transmit (i2c_instance->i2c_handle,
                               i2c_tx_data, i2c_tx_size,
                               timeout_ms);
    if (ret != SBV_OK)
        return SBV_ERROR;

    return ret;
}

int
sbv_i2c_esp32s3_master_rcv_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                                 uint8_t received_buf[], uint16_t size, uint16_t timeout_ms)
{
    return i2c_master_receive (i2c_instance->i2c_handle,
                               received_buf, size, timeout_ms);
}