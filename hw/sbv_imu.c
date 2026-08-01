#include "math.h"
#include "string.h"

#include "sbv.h"
#include "sbv_imu.h"
#include "sbv_i2c.h"

#define SBV_IMU_PI                  (3.141592654f)
#define SBV_IMU_RAD_TO_DEG          (180.0f / SBV_IMU_PI)

#ifdef SBV_MPU9250
#include "sbv_mpu9250.h"

#define SBV_IMU_PROCESS_NOISE       (0.01)
#define SBV_IMU_MEASURE_VARIANCE    (0.05)
#endif /*SBV_MPU9050*/

int
sbv_imu_init (sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle,
              sbv_imu_instance_t *imu_instance, float sampling_time_ms)
{
    int ret = SBV_OK;

    if(! i2c_instance || ! i2c_handle || ! imu_instance)
        return SBV_ERROR;

    memset(imu_instance, 0, sizeof(sbv_imu_instance_t));

    imu_instance->process_noise      = SBV_IMU_PROCESS_NOISE;
    imu_instance->measure_variance   = SBV_IMU_MEASURE_VARIANCE;
    imu_instance->sampling_time_ms   = sampling_time_ms;
    imu_instance->i2c_instance       = i2c_instance;

    imu_instance->theta.name         = SBV_IMU_THETA;
    imu_instance->phi.name           = SBV_IMU_PHI;

#ifdef SBV_MPU9250
    ret = sbv_mpu9250_init(i2c_instance, i2c_handle);
    if (ret != SBV_OK)
    {
        // LOG
        return ret;
    }
#endif /*SBV_MPU9050*/

    return ret;
}

static void
sbv_imu_sensor_read (sbv_imu_gyroscope_t* gyro, sbv_imu_accelerometer_t* acc, sbv_imu_kalman_instance_t* angle)
{
    if(!angle || !gyro || !acc)
        return;

    if (angle->name == SBV_IMU_THETA)
    {
        angle->w = (gyro->y) * (-1.0f);
        angle->est_sensor = atan2f(acc->x, acc->z) * SBV_IMU_RAD_TO_DEG;
    }
    else if(angle->name == SBV_IMU_PHI)
    {
        angle->w = gyro->x;
        angle->est_sensor = atan2f(acc->y, acc->z) * SBV_IMU_RAD_TO_DEG;
    }
}

static inline float
sbv_imu_sampling_time_to_sec (sbv_imu_instance_t *imu_instance)
{
    return imu_instance->sampling_time_ms * 0.001;
}

static void
sbv_imu_kalman_update_angle (sbv_imu_instance_t *imu_instance, sbv_imu_kalman_instance_t* angle)
{
    if(! angle || ! imu_instance)
        return;

    /* Update the variables according to Kalman filter equations */
    angle->cov_pri      = angle->cov_post + imu_instance->process_noise;
    angle->kalman_gain  = angle->cov_pri / (angle->cov_pri + imu_instance->measure_variance);
    angle->est_pri      = angle->est_post + sbv_imu_sampling_time_to_sec(imu_instance) * angle->w;
    angle->est_post     = angle->est_pri + angle->kalman_gain * (angle->est_sensor - angle->est_pri);
    angle->cov_post     = (1 - angle->kalman_gain) * angle->cov_pri;
}

int
sbv_imu_kalman_update (sbv_imu_instance_t *imu_instance)
{
    int ret = SBV_OK;

    if(! imu_instance)
        return SBV_ERROR;

    /* Read sensor value */
#ifdef SBV_MPU9250
    ret = sbv_mpu9250_read (imu_instance->i2c_instance, 
                            &(imu_instance->sbv_imu_gyro),
                            &(imu_instance->sbv_imu_acc));
    if (ret != SBV_OK)
    {
        // LOG
        return ret;
    }
#endif /*SBV_MPU9050*/

    /* Update sensor value */
    sbv_imu_sensor_read(&(imu_instance->sbv_imu_gyro), &(imu_instance->sbv_imu_acc), &imu_instance->theta);
    sbv_imu_sensor_read(&(imu_instance->sbv_imu_gyro), &(imu_instance->sbv_imu_acc), &imu_instance->phi);

    /* Kalman filter update */
    sbv_imu_kalman_update_angle(imu_instance, &imu_instance->theta);
    sbv_imu_kalman_update_angle(imu_instance, &imu_instance->phi);

    return ret;
}