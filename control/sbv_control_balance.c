#include "sbv.h"
#include "sbv_i2c.h"
#include "sbv_pid.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"
#include "sbv_control_cfg.h"
#include "sbv_control_balance.h"

#define SBV_ROBOT_WHEEL_DIAMETER     (62)
#define SBV_ROBOT_WHEEL_DISTANCE     (200)

sbv_control_balance_t   sbv_control_balance;
sbv_imu_instance_t      sbv_imu_instance;

void
sbv_control_balance_init(sbv_control_balance_t *sbv_ctrl_balance, sbv_i2c_handle_t *i2c_handle)
{
    if(!sbv_ctrl_balance || !i2c_handle)
        return;

    sbv_control_robot_speed_init(&(sbv_ctrl_balance->sbv_control_speed), SBV_ROBOT_WHEEL_DIAMETER, SBV_ROBOT_WHEEL_DISTANCE);

    sbv_imu_init(i2c_handle, &sbv_imu_instance, BALANCE_PID_SAMPLING_TIME);
    sbv_ctrl_balance->imu = &sbv_imu_instance;

    /*
     * Since the balance control require the fast reponse, but not absolutely
     * correct upright angle, we will use the PD controller for balance control
     */
    sbv_pid_init(&(sbv_ctrl_balance->balance_pid), BALANCE_PID_MAX_OUTPUT, 0,
                BALANCE_PID_KP, 0, BALANCE_PID_KD, BALANCE_PID_SAMPLING_TIME);
}

void
sbv_control_balance_update(sbv_control_balance_t *sbv_ctrl_balance)
{
    float left_motor_output, right_motor_output;

    if(! sbv_ctrl_balance || ! (sbv_ctrl_balance->sbv_control_speed.motor_left.motor)
        || ! (sbv_ctrl_balance->sbv_control_speed.motor_right.motor))
        return;

    /* Read IMU sensor and perform Kalman filter */
    sbv_imu_kalman_update(sbv_ctrl_balance->imu);

    /* Update PID control */
    sbv_pid_update_output(&(sbv_ctrl_balance->balance_pid), sbv_ctrl_balance->imu->theta.est_post);

    left_motor_output = sbv_ctrl_balance->balance_pid.output + sbv_ctrl_balance->sbv_control_speed.motor_left.motor->pwm_output;
    right_motor_output = sbv_ctrl_balance->balance_pid.output + sbv_ctrl_balance->sbv_control_speed.motor_right.motor->pwm_output;

    /* Generate PWM output to change motor speed */
    sbv_motor_output_update((sbv_ctrl_balance->sbv_control_speed.motor_left.motor), left_motor_output);
    sbv_motor_output_update((sbv_ctrl_balance->sbv_control_speed.motor_right.motor), right_motor_output);
}