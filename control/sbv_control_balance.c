#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_i2c.h"
#include "sbv_pid.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"
#include "sbv_control_cfg.h"
#include "sbv_control_balance.h"

#define SBV_ROBOT_CONTROL_MUTEX_LOCK(S) \
    sbv_rtos_mutex_lock(S->mu)
#define SBV_ROBOT_CONTROL_MUTEX_UNLOCK(S) \
    sbv_rtos_mutex_unlock(S->mu)

void
sbv_control_balance_init(sbv_control_balance_t *sbv_ctrl_balance, sbv_imu_instance_t *imu_instance,
                         sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle)
{
    if(! sbv_ctrl_balance || ! imu_instance || ! i2c_instance || ! i2c_handle)
        return;

    sbv_control_robot_speed_init(&(sbv_ctrl_balance->sbv_control_speed),
                                SBV_ROBOT_WHEEL_DIAMETER, SBV_ROBOT_WHEEL_DISTANCE);

    sbv_imu_init(i2c_instance, i2c_handle, imu_instance, BALANCE_PID_SAMPLING_TIME);
    sbv_ctrl_balance->imu = imu_instance;

    sbv_rtos_mutex_create (sbv_ctrl_balance->mu);

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
    if (! sbv_ctrl_balance || ! (sbv_ctrl_balance->imu))
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    /* Read IMU sensor and perform Kalman filter */
    sbv_imu_kalman_update(sbv_ctrl_balance->imu);

    /* Update outer balance PID control loop */
    sbv_pid_update_output(&(sbv_ctrl_balance->balance_pid), sbv_ctrl_balance->imu->theta.est_post);

    /* Feed forward this control value to inner and faster speed & twist control loop */
    sbv_control_robot_set_feed_forward(&(sbv_ctrl_balance->sbv_control_speed),
                                        sbv_ctrl_balance->balance_pid.output);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_update_speed_twist_control (sbv_control_balance_t *sbv_ctrl_balance)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_control_robot_speed_twist_update(&(sbv_ctrl_balance->sbv_control_speed));

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

uint16_t
sbv_control_balance_get_balance_sampling_time_ms (sbv_control_balance_t *sbv_ctrl_balance)
{
    uint16_t sampling_ms;

    if (! sbv_ctrl_balance)
        return 0;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sampling_ms = sbv_pid_get_sampling_time_ms (&(sbv_ctrl_balance->balance_pid));

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
    return sampling_ms;
}

uint16_t
sbv_control_balance_get_speed_sampling_time_ms (sbv_control_balance_t *sbv_ctrl_balance)
{
    uint16_t sampling_ms;

    if (! sbv_ctrl_balance)
        return 0;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sampling_ms = sbv_control_robot_speed_get_sampling_time_ms (&(sbv_ctrl_balance->sbv_control_speed));

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
    return sampling_ms;
}

void
sbv_control_balance_set_balance_pid_gain (sbv_control_balance_t *sbv_ctrl_balance,
                                          float Kp, float Ki, float Kd)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_pid_set_gain(&(sbv_ctrl_balance->balance_pid), Kp, Ki, Kd);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_set_steering_pid_gain (sbv_control_balance_t *sbv_ctrl_balance,
                                          float Kp, float Ki, float Kd)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_pid_set_gain(&(sbv_ctrl_balance->sbv_control_speed.steering_pid),
                    Kp, Ki, Kd);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_set_speed_pid_gain (sbv_control_balance_t *sbv_ctrl_balance,
                                        sbv_motor_type_t motor, float Kp, float Ki, float Kd)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    if (motor == SBV_MOTOR_LEFT)
    {
        sbv_pid_set_gain(&(sbv_ctrl_balance->sbv_control_speed.motor_left.motor_pid),
                        Kp, Ki, Kd);
    }
    else if (motor == SBV_MOTOR_RIGHT)
    {
        sbv_pid_set_gain(&(sbv_ctrl_balance->sbv_control_speed.motor_right.motor_pid),
                        Kp, Ki, Kd);
    }

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_set_balance_control_target (sbv_control_balance_t *sbv_ctrl_balance, float target)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_pid_set_target(&(sbv_ctrl_balance->balance_pid), target);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_set_robot_twist_target (sbv_control_balance_t *sbv_ctrl_balance, float target)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_control_robot_set_twist_target(&(sbv_ctrl_balance->sbv_control_speed), target);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_set_robot_speed_target (sbv_control_balance_t *sbv_ctrl_balance, float target)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_control_robot_set_speed_target(&(sbv_ctrl_balance->sbv_control_speed), target);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_set_robot_speed_twist_target (sbv_control_balance_t *sbv_ctrl_balance,
                                                 float speed, float twist)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_control_robot_set_target(&(sbv_ctrl_balance->sbv_control_speed), speed, twist);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_set_motor_speed_target (sbv_control_balance_t *sbv_ctrl_balance,
                                            sbv_motor_type_t motor, float target)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    if (motor == SBV_MOTOR_LEFT)
        sbv_pid_set_target(&(sbv_ctrl_balance->sbv_control_speed.motor_left.motor_pid), target);
    else if (motor == SBV_MOTOR_RIGHT)
        sbv_pid_set_target(&(sbv_ctrl_balance->sbv_control_speed.motor_right.motor_pid), target);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_reset_balance_pid (sbv_control_balance_t *sbv_ctrl_balance)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_pid_reset(&(sbv_ctrl_balance->balance_pid));

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_reset_steering_pid (sbv_control_balance_t *sbv_ctrl_balance)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_pid_reset(&(sbv_ctrl_balance->sbv_control_speed.steering_pid));

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_reset_speed_pid (sbv_control_balance_t *sbv_ctrl_balance,
                                    sbv_motor_type_t motor)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    if (motor == SBV_MOTOR_LEFT)
    {
        sbv_pid_reset(&(sbv_ctrl_balance->sbv_control_speed.motor_left.motor_pid));
    }
    else if (motor == SBV_MOTOR_RIGHT)
    {
        sbv_pid_reset(&(sbv_ctrl_balance->sbv_control_speed.motor_right.motor_pid));
    }

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_reset_encoder (sbv_control_balance_t *sbv_ctrl_balance)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    sbv_motor_encoder_reset((sbv_ctrl_balance->sbv_control_speed.motor_left.motor));
    sbv_motor_encoder_reset((sbv_ctrl_balance->sbv_control_speed.motor_right.motor));

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_get_encoder (sbv_control_balance_t *sbv_ctrl_balance,
                                char *data_buffer, const uint16_t len)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    snprintf(data_buffer, len, "\r\n%u,%u",
            sbv_motor_read_encoder((sbv_ctrl_balance->sbv_control_speed.motor_left.motor)),
            sbv_motor_read_encoder((sbv_ctrl_balance->sbv_control_speed.motor_right.motor)));

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_get_imu (sbv_control_balance_t *sbv_ctrl_balance,
                            char *data_buffer, const uint16_t len)
{
    if (! sbv_ctrl_balance)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    snprintf(data_buffer, len, "\r\n%.4f,%.4f,%.4f,%.4f",
            sbv_ctrl_balance->imu->theta.est_sensor,
            sbv_ctrl_balance->imu->theta.est_post,
            sbv_ctrl_balance->imu->phi.est_sensor,
            sbv_ctrl_balance->imu->phi.est_post);
    
    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}

void
sbv_control_balance_get_pid (sbv_control_balance_t *sbv_ctrl_balance, sbv_pid_t *pid,
                            char *data_buffer, const uint16_t len)
{
    if (! sbv_ctrl_balance || ! pid)
        return;

    SBV_ROBOT_CONTROL_MUTEX_LOCK(sbv_ctrl_balance);

    snprintf(data_buffer, len, "\r\n%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
            pid->target, pid->feedback, pid->output,
            pid->Kp, pid->Ki, pid->Kd);

    SBV_ROBOT_CONTROL_MUTEX_UNLOCK(sbv_ctrl_balance);
}