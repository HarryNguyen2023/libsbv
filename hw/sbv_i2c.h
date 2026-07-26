#ifndef SBV_I2C_H
#define SBV_I2C_H

#include "sbv.h"
#include "sbv_rtos.h"

typedef enum sbv_i2c_msg_t
{
    SBV_I2C_MSG_WRITE,
    SBV_I2C_MSG_READ
} sbv_i2c_msg_t;

#ifdef STM32F1xx
#include "sbv_i2c_stm32f1xx.h"
#endif

typedef struct sbv_i2c_hw_cb_t
{
    int (*sbv_i2c_master_init) (sbv_i2c_instance_t *, sbv_i2c_handle_t *);
    int (*sbv_i2c_master_send_data) (sbv_i2c_instance_t *, uint8_t, sbv_i2c_msg_t, uint8_t*, uint16_t, uint16_t);
    int (*sbv_i2c_master_rcv_data) (sbv_i2c_instance_t *, uint8_t, uint8_t[], uint16_t, uint16_t);
} sbv_i2c_hw_cb_t;

int
sbv_i2c_master_init(sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle);
int
sbv_i2c_master_send_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                          sbv_i2c_msg_t msg_type, uint8_t* i2c_tx_data,
                          uint16_t i2c_tx_size, uint16_t timeout_ms);
int
sbv_i2c_master_rcv_data (sbv_i2c_instance_t *i2c_instance, uint8_t slave_add,
                        uint8_t received_buf[], uint16_t size, uint16_t timeout_ms);

#endif /*SBV_I2C_H*/