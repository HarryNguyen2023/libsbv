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
#define SBV_MPU9250_RCV_MSG_SIZE        14

#define SBV_MPU9250_CFG_I2C_TIMEOUT     10
#define SBV_MPU9250_REQ_I2C_TIMEOUT     2
#define SBV_MPU9250_RCV_I2C_TIMEOUT     10

/* Sensitivity for SBV_MPU9250_ACCEL_CONFIG: AFS_SEL = +-8g */
#define SBV_MPU9250_ACCEL_LSB_PER_G     (4096.0f)
/* Sensitivity for SBV_MPU9250_GYRO_CONFIG: FS_SEL = +-500/s */
#define SBV_MPU9250_GYRO_LSB_PER_DPS    (65.5f)

uint8_t sbv_mpu_cfg_buffer[SBV_MPU9250_CFG_MSG_SIZE];
uint8_t sbv_mpu_rcv_buffer[SBV_MPU9250_RCV_MSG_SIZE];

static void
sbv_mpu9250_cfg_set(uint8_t reg, uint8_t data)
{
    memset(sbv_mpu_cfg_buffer, 0, sizeof(sbv_mpu_cfg_buffer));

    sbv_mpu_cfg_buffer[0] = reg;
    sbv_mpu_cfg_buffer[1] = data;
}

int
sbv_mpu9250_init(sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle)
{
    if (!i2c_instance || ! i2c_handle)
        return SBV_ERROR;

    sbv_i2c_master_init (i2c_instance, i2c_handle);
    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_PWR_MGMT_1, SBV_MPU9250_PWR_CONFIG);
    sbv_i2c_master_send_data(i2c_instance, SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE, SBV_MPU9250_CFG_I2C_TIMEOUT);

    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_CONFIG, SBV_MPU9250_GENERAL_CONFIG);
    sbv_i2c_master_send_data(i2c_instance, SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE, SBV_MPU9250_CFG_I2C_TIMEOUT);

    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_GYRO_CONFIG, SBV_MPU9250_GYRO_CONFIG);
    sbv_i2c_master_send_data(i2c_instance, SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE, SBV_MPU9250_CFG_I2C_TIMEOUT);

    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_ACCEL_CONFIG, SBV_MPU9250_ACCEL_CONFIG);
    sbv_i2c_master_send_data(i2c_instance, SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE, SBV_MPU9250_CFG_I2C_TIMEOUT);

    sbv_mpu9250_cfg_set(SBV_MPU9250_REG_ACCEL_CONFIG_2, SBV_MPU9250_ACCEL_CONFIG_2);
    sbv_i2c_master_send_data(i2c_instance, SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                             sbv_mpu_cfg_buffer, SBV_MPU9250_CFG_MSG_SIZE, SBV_MPU9250_CFG_I2C_TIMEOUT);

    return SBV_OK;
}

static int16_t
sbv_mpu9250_read_raw16 (uint8_t high_byte, uint8_t low_byte)
{
    return (int16_t)((high_byte << 8) | low_byte);
}

static void
sbv_mpu9250_read_sensor(sbv_imu_gyroscope_t *gyro, sbv_imu_accelerometer_t *accel,
                        uint8_t* sbv_i2c_rx_buffer, uint16_t sbv_i2c_rx_size)
{
    if (!gyro || !accel || !sbv_i2c_rx_buffer)
        return;

    if (sbv_i2c_rx_size < SBV_MPU9250_RCV_MSG_SIZE)
    {
        /* LOG */
        return;
    }

    accel->x = (float) sbv_mpu9250_read_raw16(sbv_i2c_rx_buffer[0], sbv_i2c_rx_buffer[1])
                / SBV_MPU9250_ACCEL_LSB_PER_G;
    accel->y = (float) sbv_mpu9250_read_raw16(sbv_i2c_rx_buffer[2], sbv_i2c_rx_buffer[3])
                / SBV_MPU9250_ACCEL_LSB_PER_G;
    accel->z = (float) sbv_mpu9250_read_raw16(sbv_i2c_rx_buffer[4], sbv_i2c_rx_buffer[5])
                / SBV_MPU9250_ACCEL_LSB_PER_G;

    gyro->x = (float) sbv_mpu9250_read_raw16(sbv_i2c_rx_buffer[8], sbv_i2c_rx_buffer[9])
                / SBV_MPU9250_GYRO_LSB_PER_DPS;
    gyro->y = (float) sbv_mpu9250_read_raw16(sbv_i2c_rx_buffer[10], sbv_i2c_rx_buffer[11])
                / SBV_MPU9250_GYRO_LSB_PER_DPS;
    gyro->z = (float) sbv_mpu9250_read_raw16(sbv_i2c_rx_buffer[12], sbv_i2c_rx_buffer[13])
                / SBV_MPU9250_GYRO_LSB_PER_DPS;
}

int
sbv_mpu9250_read(sbv_i2c_instance_t *i2c_instance,
                 sbv_imu_gyroscope_t *gyro, sbv_imu_accelerometer_t *accel)
{
    int ret = SBV_OK, recv_byte = 0;
    uint8_t try_num = 0, read_reg;

    if(! i2c_instance || !gyro || !accel)
        return SBV_ERROR;

    read_reg = SBV_MPU9250_REG_ACCEL_XOUT_H;
    /* Start to read from the  SBV_MPU9250_REG_ACCEL_XOUT_H register */
    while (try_num++ < SBV_MPU_9250_MAX_WRITE_TRY)
    {
        
        ret = sbv_i2c_master_send_data(i2c_instance, SBV_MPU9250_I2C_ADDR, SBV_I2C_MSG_WRITE,
                                       &read_reg, 1, SBV_MPU9250_REQ_I2C_TIMEOUT);
        if (ret == SBV_OK)
            break;
    }

    /* Fail to request data from MPU9250 */
    if (try_num > SBV_MPU_9250_MAX_WRITE_TRY)
    {
        // LOG
        return SBV_ERROR;
    }

    /* Read all 14 bytes of data (6 Accel bytes, 2 Temp bytes and 6 Gyro bytes) */
    recv_byte = sbv_i2c_master_rcv_data (i2c_instance, SBV_MPU9250_I2C_ADDR,
                                        sbv_mpu_rcv_buffer, SBV_MPU9250_RCV_MSG_SIZE,
                                        SBV_MPU9250_RCV_I2C_TIMEOUT);
    if (recv_byte <= 0)
    {
        // LOG
        return SBV_ERROR;
    }

    sbv_mpu9250_read_sensor(gyro, accel, sbv_mpu_rcv_buffer, recv_byte);
    return SBV_OK;
}