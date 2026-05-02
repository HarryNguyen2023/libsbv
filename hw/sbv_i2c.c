#include <string.h>
#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_i2c.h"

sbv_rtos_mutex_t SBV_I2C_TX_BUFFER_MUTEX;
sbv_rtos_mutex_t SBV_I2C_RX_BUFFER_MUTEX;

sbv_i2c_hw_cb_t sbv_i2c_hw_cb = {
#ifdef STM32F1xx
    .sbv_i2c_master_init        = sbv_i2c_stm32f1xx_master_init,
    .sbv_i2c_master_send_data   = sbv_i2c_stm32f1xx_master_send_data,
    .sbv_i2c_master_rcv_data    = sbv_i2c_stm32f1xx_master_rcv_data,
#endif /* STM32F1xx */
};

void
sbv_i2c_master_init(sbv_i2c_handle_t *i2c_handle)
{
    if (sbv_i2c_hw_cb.sbv_i2c_master_init)
        (sbv_i2c_hw_cb.sbv_i2c_master_init) (i2c_handle);
}

int
sbv_i2c_master_send_data(uint8_t slave_add, sbv_i2c_msg_t msg_type,
                         uint8_t* i2c_tx_data, uint16_t i2c_tx_size)
{
    if (sbv_i2c_hw_cb.sbv_i2c_master_send_data)
        return (sbv_i2c_hw_cb.sbv_i2c_master_send_data) (slave_add, msg_type,
                                                        i2c_tx_data, i2c_tx_size);
    return 0;
}

uint8_t*
sbv_i2c_master_rcv_data (uint8_t slave_add, uint16_t *size)
{
    if (sbv_i2c_hw_cb.sbv_i2c_master_rcv_data)
        return (sbv_i2c_hw_cb.sbv_i2c_master_rcv_data) (slave_add, size);

    *size = 0;
    return NULL;
}