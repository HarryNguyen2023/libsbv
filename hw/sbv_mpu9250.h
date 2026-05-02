#ifndef SBV_MPU9250_H
#define SBV_MPU9250_H

#include "sbv.h"
#include "sbv_i2c.h"

#define SBV_MPU9250_I2C_ADDR    (0x34)

/* MPU 9250 register addresses */
#define SBV_MPU9250_REG_CONFIG          (0x1A)
#define SBV_MPU9250_REG_GYRO_CONFIG     (0x1B)
#define SBV_MPU9250_REG_ACCEL_CONFIG    (0x1C)
#define SBV_MPU9250_REG_ACCEL_CONFIG_2  (0x1D)
#define SBV_MPU9250_REG_PWR_MGMT_1      (0x6B)
#define SBV_MPU9250_REG_PWR_MGMT_2      (0x6C)

#define SBV_MPU9250_REG_ACCEL_XOUT_H    (0x3B)
#define SBV_MPU9250_REG_ACCEL_XOUT_L    (0x3C)
#define SBV_MPU9250_REG_ACCEL_YOUT_H    (0x3D)
#define SBV_MPU9250_REG_ACCEL_YOUT_L    (0x3E)
#define SBV_MPU9250_REG_ACCEL_ZOUT_H    (0x3F)
#define SBV_MPU9250_REG_ACCEL_ZOUT_L    (0x40)
#define SBV_MPU9250_REG_TEMP_OUT_H      (0x41)
#define SBV_MPU9250_REG_TEMP_OUT_L      (0x42)
#define SBV_MPU9250_REG_GYRO_XOUT_H     (0x43)
#define SBV_MPU9250_REG_GYRO_XOUT_L     (0x44)
#define SBV_MPU9250_REG_GYRO_YOUT_H     (0x45)
#define SBV_MPU9250_REG_GYRO_YOUT_L     (0x46)
#define SBV_MPU9250_REG_GYRO_ZOUT_H     (0x47)
#define SBV_MPU9250_REG_GYRO_ZOUT_L     (0x48)

void
sbv_mpu9250_init(sbv_i2c_handle_t *i2c_handle);
void
sbv_mpu9250_read(sbv_imu_gyroscope_t *gyro, sbv_imu_accelerometer_t *accel);

#endif /*SBV_MPU9250_H*/