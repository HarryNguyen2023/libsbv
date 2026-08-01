#include "sbv.h"
#include "sbv_i2c.h"
#include "sbv_pid.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"
#include "sbv_control_cfg.h"
#include "sbv_control_balance.h"

#define SBV_ROBOT_WHEEL_DIAMETER     (62)
#define SBV_ROBOT_WHEEL_DISTANCE     (200)
void
sbv_control_balance_init(sbv_control_balance_t *sbv_ctrl_balance, sbv_imu_instance_t *imu_instance,
                         sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle)
{
    if(! sbv_ctrl_balance || ! i2c_instance || ! i2c_handle)
        return;

    sbv_control_robot_speed_init(&(sbv_ctrl_balance->sbv_control_speed),
                                SBV_ROBOT_WHEEL_DIAMETER, SBV_ROBOT_WHEEL_DISTANCE);

    sbv_imu_init(i2c_instance, i2c_handle, imu_instance, BALANCE_PID_SAMPLING_TIME);
    sbv_ctrl_balance->imu = imu_instance;

    /*
     * Since the balance control require the fast reponse, but not absolutely
     * correct upright angle, we will use the PD controller for balance control
     */
    sbv_pid_init(&(sbv_ctrl_balance->balance_pid),
                BALANCE_PID_MAX_OUTPUT, BALANCE_PID_MIN_OUTPUT,
                BALANCE_PID_KP, 0, BALANCE_PID_KD, BALANCE_PID_SAMPLING_TIME);
}

void
sbv_control_balance_update(sbv_control_balance_t *sbv_ctrl_balance)
{
    float left_motor_output, right_motor_output;

    if (! sbv_ctrl_balance || ! (sbv_ctrl_balance->imu))
        return;

    /* Read IMU sensor and perform Kalman filter */
    sbv_imu_kalman_update(sbv_ctrl_balance->imu);

    /* Update outer balance PID control loop */
    sbv_pid_update_output(&(sbv_ctrl_balance->balance_pid), sbv_ctrl_balance->imu->theta.est_post);

    /* Feed forward this control value to inner and faster speed & twist control loop */
    sbv_control_robot_set_feed_forward(&(sbv_ctrl_balance->sbv_control_speed),
                                        sbv_ctrl_balance->balance_pid.output);
}

void
sbv_control_balance_update_speed_twist_control (sbv_control_balance_t *sbv_ctrl_balance)
{
    if (! sbv_ctrl_balance)
        return;

    sbv_control_robot_speed_twist_update(&(sbv_ctrl_balance->sbv_control_speed));
}

uint16_t
sbv_control_balance_get_balance_sampling_time_ms (sbv_control_balance_t *sbv_ctrl_balance)
{
    if (! sbv_ctrl_balance)
        return 0;

    return sbv_pid_get_sampling_time_ms (&(sbv_ctrl_balance->balance_pid));
}

uint16_t
sbv_control_balance_get_speed_sampling_time_ms (sbv_control_balance_t *sbv_ctrl_balance)
{
    if (! sbv_ctrl_balance)
        return 0;

    return sbv_control_robot_speed_get_sampling_time_ms (&(sbv_ctrl_balance->sbv_control_speed));
}