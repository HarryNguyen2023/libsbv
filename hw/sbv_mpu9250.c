#include <string.h>

#include "sbv.h"
#include "sbv_i2c.h"
#include "sbv_imu.h"
#include "sbv_mpu9250.h"

#define SBV_MPU9250_GENERAL_CONFIG      (0x05)
#define SBV_MPU9250_GYRO_CONFIG         (0x08)
#define SBV_MPU9250_ACCEL_CONFIG        (0x10)
#define SBV_MPU9250_ACCEL_CONFIG_2      (0x05)
#define SBV_MPU9250_PWR_CONFIG          (0x01)

#define SBV_MPU9250_CFG_MSG_SIZE        2

uint8_t sbv_mpu_cfg_buffer[2];

static void
sbv_mpu9250_cfg_set(uint8_t reg, uint8_t data)
{
    memset(sbv_mpu_cfg_buffer, 0, sizeof(sbv_mpu_cfg_buffer));

    sbv_mpu_cfg_buffer[0] = reg;
    sbv_mpu_cfg_buffer[1] = data;
}

void
sbv_mpu9250_init(sbv_i2c_handle_t *i2c_handle)
{
    if (! i2c_handle)
        return;

    sbv_i2c_master_init (i2c_handle);
    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_PWR_MGMT_1, SBV_MPU9250_PWR_CONFIG);
    sbv_i2c_master_send_data(SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE);

    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_CONFIG, SBV_MPU9250_GENERAL_CONFIG);
    sbv_i2c_master_send_data (SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                              sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE);

    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_GYRO_CONFIG, SBV_MPU9250_GYRO_CONFIG);
    sbv_i2c_master_send_data(SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE);

    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_ACCEL_CONFIG, SBV_MPU9250_ACCEL_CONFIG);
    sbv_i2c_master_send_data(SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE);

    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_ACCEL_CONFIG_2, SBV_MPU9250_ACCEL_CONFIG_2);
    sbv_i2c_master_send_data(SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE);
}

static void
sbv_mpu9250_read_sensor(sbv_imu_gyroscope_t *gyro, sbv_imu_accelerometer_t *accel,
                        uint8_t* sbv_i2c_rx_buffer, uint16_t sbv_i2c_rx_size)
{
    if (!gyro || !accel || !sbv_i2c_rx_buffer)
        return;

    if (sbv_i2c_rx_size < SBV_I2C_RX_BUFFER_SIZE)
    {
        /* LOG */
        return;
    }

    accel->x = (float) ((sbv_i2c_rx_buffer[0] << 8) | sbv_i2c_rx_buffer[1]);
    accel->y = (float) ((sbv_i2c_rx_buffer[2] << 8) | sbv_i2c_rx_buffer[3]);
    accel->z = (float) ((sbv_i2c_rx_buffer[4] << 8) | sbv_i2c_rx_buffer[5]);

    gyro->x = (float) ((sbv_i2c_rx_buffer[8] << 8)  | sbv_i2c_rx_buffer[9]);
    gyro->y = (float) ((sbv_i2c_rx_buffer[10] << 8) | sbv_i2c_rx_buffer[11]);
    gyro->z = (float) ((sbv_i2c_rx_buffer[12] << 8) | sbv_i2c_rx_buffer[13]);
}

void
sbv_mpu9250_read(sbv_imu_gyroscope_t *gyro, sbv_imu_accelerometer_t *accel)
{
    uint8_t* sbv_i2c_rx_buffer = NULL;
    uint16_t sbv_i2c_rx_size = 0;

    if(!gyro || !accel)
        return;

    /* Start to read from the  SBV_MPU9250_REG_ACCEL_XOUT_H register */
    sbv_i2c_master_send_data(SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_READ,
                             (uint8_t*) SBV_MPU9250_REG_ACCEL_XOUT_H, 1);

    /* Read all 14 bytes of data (6 Accel bytes, 2 Temp bytes and 6 Gyro bytes) */
    sbv_i2c_rx_buffer = sbv_i2c_master_rcv_data (SBV_MPU9250_I2C_ADDR, &sbv_i2c_rx_size);
    if (sbv_i2c_rx_buffer && sbv_i2c_rx_size)
        sbv_mpu9250_read_sensor(gyro, accel, sbv_i2c_rx_buffer, sbv_i2c_rx_size);
}


