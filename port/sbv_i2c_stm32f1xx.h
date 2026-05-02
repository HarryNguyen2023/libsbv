#ifndef __SBV_I2C_STM32F1XX_H__
#define __SBV_I2C_STM32F1XX_H__

#include "sbv.h"
#include "sbv_rtos.h"

#ifdef STM32F1xx

#include "stm32f1xx_hal_i2c.h"

#define SBV_I2C_TX_BUFFER_SIZE      50
/* 
 * Since we mostly use I2C to read gyro, temp and accelerometer value,
 * we normally need 50 bytes buffer
 */
#define SBV_I2C_RX_BUFFER_SIZE      14

typedef I2C_HandleTypeDef   sbv_i2c_handle_t;

typedef struct sbv_i2c_instance_t
{
    uint8_t                 i2c_active;
    uint8_t*                i2c_rx_buffer;
    uint8_t*                i2c_tx_buffer;
    sbv_i2c_handle_t*       i2c_handle;
    sbv_rtos_task_handle_t  i2c_rx_notify_task;
    uint16_t                i2c_rx_buffer_pos;
    uint16_t                i2c_rx_isr_size;
} sbv_i2c_instance_t;

void
sbv_i2c_stm32f1xx_master_init(sbv_i2c_handle_t *i2c_handle);
int
sbv_i2c_stm32f1xx_master_send_data(uint8_t slave_add, sbv_i2c_msg_t msg_type,
                                   uint8_t* i2c_tx_data, uint16_t i2c_tx_size);
uint8_t *
sbv_i2c_stm32f1xx_master_rcv_data (uint8_t slave_add, uint16_t *size);
#endif /* STM32F1xx */
#endif /* __SBV_I2C_STM32F1XX_H__ */