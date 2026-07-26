#ifndef __SBV_I2C_STM32F1XX_H__
#define __SBV_I2C_STM32F1XX_H__

#include "sbv.h"
#include "sbv_rtos.h"

#ifdef STM32F1xx

#include "stm32f1xx_hal_i2c.h"

#define SBV_I2C_MAX_CHANNEL         2
/* 
 * Since we mostly use I2C to read gyro,
 * temp and accelerometer value,
 * we normally need 14 bytes buffer
 */
#define SBV_I2C_RX_BUFFER_SIZE 50

typedef I2C_HandleTypeDef   sbv_i2c_handle_t;

typedef struct sbv_i2c_instance_t
{
    uint8_t                 i2c_rx_buffer[SBV_I2C_RX_BUFFER_SIZE];
    sbv_i2c_handle_t*       i2c_handle;
    sbv_rtos_task_handle_t  i2c_rx_notify_task;
    uint16_t                i2c_rx_buffer_pos;
    sbv_rtos_mutex_t        mutex;
} sbv_i2c_instance_t;

struct sbv_i2c_instance_list_t {
    sbv_i2c_instance_t* list[SBV_I2C_MAX_CHANNEL];
};

int
sbv_i2c_stm32f1xx_master_init(sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle);
int
sbv_i2c_stm32f1xx_master_send_data (sbv_i2c_instance_t *i2c_instance,
                                    uint8_t slave_add, sbv_i2c_msg_t msg_type,
                                    uint8_t* i2c_tx_data, uint16_t i2c_tx_size);
int
sbv_i2c_stm32f1xx_master_rcv_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                                   uint8_t received_buf[], uint16_t size);
#endif /* STM32F1xx */
#endif /* __SBV_I2C_STM32F1XX_H__ */