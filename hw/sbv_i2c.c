#include <string.h>
#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_i2c.h"

sbv_i2c_hw_cb_t sbv_i2c_hw_cb = {
#ifdef STM32F1xx
    .sbv_i2c_master_init        = sbv_i2c_stm32f1xx_master_init,
    .sbv_i2c_master_send_data   = sbv_i2c_stm32f1xx_master_send_data,
    .sbv_i2c_master_rcv_data    = sbv_i2c_stm32f1xx_master_rcv_data,
#endif /* STM32F1xx */
};

int
sbv_i2c_master_init(sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle)
{
    if (sbv_i2c_hw_cb.sbv_i2c_master_init)
        (sbv_i2c_hw_cb.sbv_i2c_master_init) (i2c_instance, i2c_handle);

    return SBV_ERROR;
}

int
sbv_i2c_master_send_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                          sbv_i2c_msg_t msg_type, uint8_t* i2c_tx_data,
                          uint16_t i2c_tx_size, uint16_t timeout_ms)
{
    if (sbv_i2c_hw_cb.sbv_i2c_master_send_data)
        return (sbv_i2c_hw_cb.sbv_i2c_master_send_data) (i2c_instance, slave_add, msg_type,
                                                        i2c_tx_data, i2c_tx_size, timeout_ms);
    return SBV_ERROR;
}

int
sbv_i2c_master_rcv_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                        uint8_t received_buf[], uint16_t size, uint16_t timeout_ms)
{
    if (sbv_i2c_hw_cb.sbv_i2c_master_rcv_data)
        return (sbv_i2c_hw_cb.sbv_i2c_master_rcv_data) (i2c_instance, slave_add,
                                                        received_buf, size, timeout_ms);

    return SBV_ERROR;
}