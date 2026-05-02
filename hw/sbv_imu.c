#include "math.h"
#include "string.h"

#include "sbv.h"
#include "sbv_imu.h"
#include "sbv_i2c.h"

#define SBV_IMU_PI                  (3.141592654)
#define SBV_IMU_GRAV_ACC            (9.8)

#ifdef SBV_MPU9250
#include "sbv_mpu9250.h"

#define SBV_IMU_PROCESS_NOISE       (0.01)
#define SBV_IMU_MEASURE_VARIANCE    (0.05)
#endif /*SBV_MPU9050*/

/************************* Global variable definition **************************/
static sbv_imu_gyroscope_t     sbv_imu_gyro;
static sbv_imu_accelerometer_t sbv_imu_acc;

void
sbv_imu_init (sbv_i2c_handle_t *i2c_handle, sbv_imu_instance_t *imu_instance, float sampling_time_ms)
{
    if(!i2c_handle)
        return;

    memset(imu_instance, 0, sizeof(sbv_imu_instance_t));

    imu_instance->process_noise      = SBV_IMU_PROCESS_NOISE;
    imu_instance->measure_variance   = SBV_IMU_MEASURE_VARIANCE;
    imu_instance->sampling_time_ms   = sampling_time_ms;
    imu_instance->i2c_handle         = i2c_handle;

    imu_instance->theta.name         = SBV_IMU_THETA;
    imu_instance->phi.name           = SBV_IMU_PHI;

#ifdef SBV_MPU9250
    sbv_mpu9250_init(i2c_handle);
#endif /*SBV_MPU9050*/
}

static void
sbv_imu_sensor_read (sbv_imu_gyroscope_t* gyro, sbv_imu_accelerometer_t* acc, sbv_imu_kalman_instance_t* angle)
{
    if(!angle || !gyro || !acc)
        return;

    if (angle->name == SBV_IMU_THETA)
    {
        angle->w = (gyro->y) * (-1.0);
        angle->est_sensor = ((atan2(acc->x / SBV_IMU_GRAV_ACC, acc->z / SBV_IMU_GRAV_ACC) / (2 * SBV_IMU_PI)) * 360);
    }
    else if(angle->name == SBV_IMU_PHI)
    {
        angle->w = gyro->x;
        angle->est_sensor = ((atan2(acc->y / SBV_IMU_GRAV_ACC, acc->z / SBV_IMU_GRAV_ACC) / (2 * SBV_IMU_PI)) * 360);
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
    if(!angle || !imu_instance)
        return;
    
    /* Update the variables according to Kalman filter equations */
    angle->cov_pri      = angle->cov_post + imu_instance->process_noise;
    angle->kalman_gain  = angle->cov_pri / (angle->cov_pri + imu_instance->measure_variance);
    angle->est_pri      = angle->est_post + sbv_imu_sampling_time_to_sec(imu_instance) * angle->w;
    angle->est_post     = angle->est_pri + angle->kalman_gain * (angle->est_sensor - angle->est_pri);
    angle->cov_post     = (1 - angle->kalman_gain) * angle->cov_pri;
}

void
sbv_imu_kalman_update (sbv_imu_instance_t *imu_instance)
{
    if(!imu_instance)
        return;

    /* Read sensor value */
#ifdef SBV_MPU9250
    sbv_mpu9250_read(&sbv_imu_gyro, &sbv_imu_acc);
#endif /*SBV_MPU9050*/

    /* Update sensor value */
    sbv_imu_sensor_read(&sbv_imu_gyro, &sbv_imu_acc, &imu_instance->theta);
    sbv_imu_sensor_read(&sbv_imu_gyro, &sbv_imu_acc, &imu_instance->phi);

    /* Kalman filter update */
    sbv_imu_kalman_update_angle(imu_instance, &imu_instance->theta);
    sbv_imu_kalman_update_angle(imu_instance, &imu_instance->phi);
}