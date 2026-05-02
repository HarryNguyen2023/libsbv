#ifndef SBV_IMU_H
#define SBV_IMU_H

#include "sbv.h"
#include "sbv_i2c.h"

typedef struct sbv_imu_accelerometer_t
{
    float x;
    float y;
    float z;
} sbv_imu_accelerometer_t;

typedef struct sbv_imu_gyroscope_t
{
    float x;
    float y;
    float z;
} sbv_imu_gyroscope_t;

typedef enum sbv_imu_kalman_type_t
{
    SBV_IMU_THETA,
    SBV_IMU_PHI
} sbv_imu_kalman_type_t;

typedef struct sbv_imu_kalman_instance_t
{
    sbv_imu_kalman_type_t name;
    float w;
    float est_sensor;
    float est_pri;
    float est_post;
    float cov_pri;
    float cov_post;
    float kalman_gain;
} sbv_imu_kalman_instance_t;

typedef struct sbv_imu_instance_t
{
    sbv_imu_kalman_instance_t   theta;
    sbv_imu_kalman_instance_t   phi;
    float                       process_noise;
    float                       measure_variance;
    uint16_t                    sampling_time_ms;
    sbv_i2c_handle_t            *i2c_handle;
} sbv_imu_instance_t;

void
sbv_imu_init (sbv_i2c_handle_t *i2c_handle, sbv_imu_instance_t *imu_instance, float sampling_time_ms);
void
sbv_imu_kalman_update (sbv_imu_instance_t *imu_instance);

#endif /*SBV_IMU_H*/