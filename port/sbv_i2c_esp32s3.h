#ifndef __SBV_I2C_ESP32S3_H__
#define __SBV_I2C_ESP32S3_H__

#include "sbv.h"
#include "sbv_rtos.h"

#ifdef ESP32xx_IDF
#include "driver/i2c_master.h"

#define SBV_I2C_MAX_CHANNEL         2

typedef i2c_port_t                sbv_i2c_handle_t;
typedef i2c_master_dev_handle_t   sbv_i2c_dev_handle_t;

typedef struct sbv_i2c_instance_t
{
    sbv_i2c_dev_handle_t    i2c_handle;
    sbv_rtos_mutex_t        mutex;
} sbv_i2c_instance_t;

struct sbv_i2c_instance_list_t {
    sbv_i2c_instance_t* list[SBV_I2C_MAX_CHANNEL];
};

int
sbv_i2c_esp32s3_master_init(sbv_i2c_instance_t *i2c_instance,
                            sbv_i2c_handle_t *i2c_handle,
                            uint8_t slave_addr);
int
sbv_i2c_esp32s3_master_send_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                                  sbv_i2c_msg_t msg_type, uint8_t* i2c_tx_data,
                                  uint16_t i2c_tx_size, uint16_t timeout_ms);
int
sbv_i2c_esp32s3_master_rcv_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                                 uint8_t received_buf[], uint16_t size, uint16_t timeout_ms);
#endif /* STM32F1xx */
#endif /* __SBV_I2C_ESP32S3_H__ */