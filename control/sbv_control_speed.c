#include "sbv.h"
#include "sbv_pid.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"
#include "sbv_control_cfg.h"
#include "sbv_control_speed.h"

#define SBV_PI      (3.14159)
#define SBV_MS_IN_S (1000)

/************************** Global variable definition *************************/
extern sbv_motor_t motor_left;
extern sbv_motor_t motor_right;

static void
sbv_control_motor_speed_init(sbv_control_motor_speed_t* motor_speed)
{
    if(! motor_speed)
        return;

    /* 
     * Normally for speed control of motor we are more interested in
     * the accuracy of the output value not the setlling time, thus we
     * mostly use PI controller to avoid noisy effect of the derivative
     */
    if(motor_speed->name == SBV_MOTOR_LEFT)
    {
        motor_speed->motor = &motor_left;
        if (motor_speed->motor == NULL)
        {
            /* LOG */
            return;
        }
        sbv_motor_init(motor_speed->motor);
        sbv_pid_init(&motor_speed->motor_pid, motor_speed->motor->max_pwm, 0,
                     MOTOR_LEFT_PID_KP, MOTOR_LEFT_PID_KI, 0.0, MOTOR_PID_SAMPLING_TIME_MS);
    }
    else if(motor_speed->name == SBV_MOTOR_RIGHT)
    {
        motor_speed->motor = &motor_right;
        if (motor_speed->motor == NULL)
        {
            /* LOG */
            return;
        }
        sbv_motor_init(motor_speed->motor);
        sbv_pid_init(&motor_speed->motor_pid, motor_speed->motor->max_pwm, 0,
                     MOTOR_RIGHT_PID_KP, MOTOR_RIGHT_PID_KI, 0.0, MOTOR_PID_SAMPLING_TIME_MS);
    }
}

void
sbv_control_robot_speed_init(sbv_control_robot_speed_t *sbv_speed_ctrl, float wheel_diameter, float wheel_distance)
{
    if(! sbv_speed_ctrl)
        return;

    sbv_speed_ctrl->speed = 0;
    sbv_speed_ctrl->twist = 0;

    sbv_speed_ctrl->wheel_diameter = wheel_diameter;
    sbv_speed_ctrl->wheel_distance = wheel_distance;

    sbv_speed_ctrl->motor_left.name = SBV_MOTOR_LEFT;
    sbv_control_motor_speed_init(&sbv_speed_ctrl->motor_left);

    sbv_speed_ctrl->motor_right.name = SBV_MOTOR_RIGHT;
    sbv_control_motor_speed_init(&sbv_speed_ctrl->motor_right);

    /*
     * Here we add a PD loop to control the steering of the robot
     * due to the inreliability of the robot mechanical system in reality
     */
    sbv_pid_init(&(sbv_speed_ctrl->steering_pid), STEERING_PID_MAX_OUTPUT,
                0, STERRING_PID_KP, 0, STEERING_PID_KD, STEERING_PID_SAMPLING_TIME);
}

static void
sbv_control_motor_speed_input_handle(sbv_control_motor_speed_t *motor_speed, float speed_rps)
{
    float motor_max_rps;

    if(! motor_speed || ! motor_speed->motor || ! speed_rps)
        return;

    motor_max_rps = RPM_TO_RPS(motor_speed->motor->max_speed_rpm);

    if(speed_rps > motor_max_rps)
        speed_rps = motor_max_rps;
    else if(speed_rps < (-1.0) * motor_max_rps)
        speed_rps = (-1.0) * motor_max_rps;

    motor_speed->motor_pid.target = (speed_rps * motor_speed->motor->encoder_rev)
                                    * (motor_speed->motor_pid.sampling_time_ms / SBV_MS_IN_S);
}

static void
sbv_control_convert_motor_speed_target(sbv_control_robot_speed_t *sbv_speed_ctrl)
{
    float motor_rps;

    if(!sbv_speed_ctrl)
        return;

    /* Convert them to rps */
    motor_rps = ROBOT_SPEED_TO_MOTOR_SPEED(sbv_speed_ctrl->speed, sbv_speed_ctrl->wheel_diameter);

    sbv_control_motor_speed_input_handle(&sbv_speed_ctrl->motor_left, motor_rps);
    sbv_control_motor_speed_input_handle(&sbv_speed_ctrl->motor_right, motor_rps);
}

/*
 *@brief: This function used to set the target speed for robot in (m/s) unit
 * before convert them to pulse per control frame in motor speed PID control
*/
void
sbv_control_robot_set_speed_target(sbv_control_robot_speed_t *sbv_speed_ctrl, float speed)
{
    if(!sbv_speed_ctrl)
        return;

    sbv_speed_ctrl->speed = speed;
    sbv_control_convert_motor_speed_target(sbv_speed_ctrl);
}

static void
sbv_control_convert_motor_twsit_target(sbv_control_robot_speed_t *sbv_speed_ctrl)
{
    if(! sbv_speed_ctrl || ! sbv_speed_ctrl->motor_left.motor)
        return;

    /* Convert them to rps */
    sbv_speed_ctrl->steering_pid.target = (sbv_speed_ctrl->twist * sbv_speed_ctrl->wheel_distance)
                                          / (SBV_PI * sbv_speed_ctrl->wheel_diameter);

    sbv_speed_ctrl->steering_pid.target *= (sbv_speed_ctrl->motor_left.motor->encoder_rev)
                                            * (sbv_speed_ctrl->steering_pid.sampling_time_ms / SBV_MS_IN_S);
}

/*
 *@brief: This function used to set the target twist for robot in (rad/s) unit
 * before convert them to pulse per control frame in motor speed PID control
*/
void
sbv_control_robot_set_twist_target(sbv_control_robot_speed_t *sbv_speed_ctrl, float twsit)
{
    if(!sbv_speed_ctrl)
        return;

    sbv_speed_ctrl->twist = twsit;
    sbv_control_convert_motor_twsit_target(sbv_speed_ctrl);
}

void
sbv_control_robot_set_target(sbv_control_robot_speed_t *sbv_speed_ctrl, float speed, float twist)
{
    if(!sbv_speed_ctrl)
        return;

    sbv_control_robot_set_speed_target(sbv_speed_ctrl, speed);
    sbv_control_robot_set_twist_target(sbv_speed_ctrl, twist);
}

static void
sbv_control_motor_speed_update_test(sbv_control_motor_speed_t* motor_speed)
{
    if(! motor_speed || ! motor_speed->motor)
        return;

    /* Read encoders value */
    sbv_motor_get_speed((motor_speed->motor));

    /* Update PID control */
    sbv_pid_update_output(&(motor_speed->motor_pid), motor_speed->motor->encoder_shift);

    /* Generate output PWM to change motor speed */
    sbv_motor_output_update((motor_speed->motor), motor_speed->motor_pid.output);
}

void
sbv_control_robot_speed_update_test(sbv_control_robot_speed_t *sbv_speed_ctrl)
{
    if(!sbv_speed_ctrl)
        return;

    sbv_control_motor_speed_update_test(&(sbv_speed_ctrl->motor_left));
    sbv_control_motor_speed_update_test(&(sbv_speed_ctrl->motor_right));
}

static void
sbv_control_motor_speed_update(sbv_control_motor_speed_t* motor_speed)
{
    if(! motor_speed || ! motor_speed->motor)
        return;

    /* Read encoders value */
    sbv_motor_get_speed((motor_speed->motor));

    /* Update PID control */
    sbv_pid_update_output(&(motor_speed->motor_pid), motor_speed->motor->encoder_shift);
}

void
sbv_control_robot_speed_update(sbv_control_robot_speed_t *sbv_speed_ctrl)
{
    if(!sbv_speed_ctrl)
        return;

    sbv_control_motor_speed_update(&(sbv_speed_ctrl->motor_left));
    sbv_control_motor_speed_update(&(sbv_speed_ctrl->motor_right));
}

void
sbv_control_robot_twist_update_test(sbv_control_robot_speed_t *sbv_speed_ctrl)
{
    float wheel_diff_pulse_per_frame;

    if(! sbv_speed_ctrl || ! sbv_speed_ctrl->motor_right.motor || ! sbv_speed_ctrl->motor_left.motor)
        return;

    /* Read 2 wheel encoders value */
    sbv_motor_get_speed((sbv_speed_ctrl->motor_right.motor));
    sbv_motor_get_speed((sbv_speed_ctrl->motor_left.motor));

    wheel_diff_pulse_per_frame = (sbv_speed_ctrl->motor_right.motor->encoder_shift) -
                                 (sbv_speed_ctrl->motor_left.motor->encoder_shift);
    /* Update PID control */
    sbv_pid_update_output(&(sbv_speed_ctrl->steering_pid), wheel_diff_pulse_per_frame);

    /* Generate output PWM to change motor speed */
    sbv_motor_output_update((sbv_speed_ctrl->motor_left.motor), (-1.0) * sbv_speed_ctrl->steering_pid.output);
    sbv_motor_output_update((sbv_speed_ctrl->motor_right.motor), sbv_speed_ctrl->steering_pid.output);
}

void
sbv_control_robot_twist_update(sbv_control_robot_speed_t *sbv_speed_ctrl)
{
    float wheel_diff_pulse_per_frame;

    if(! sbv_speed_ctrl || ! sbv_speed_ctrl->motor_right.motor || ! sbv_speed_ctrl->motor_left.motor)
        return;

    /* Read 2 wheel encoders value */
    sbv_motor_get_speed((sbv_speed_ctrl->motor_right.motor));
    sbv_motor_get_speed((sbv_speed_ctrl->motor_left.motor));

    wheel_diff_pulse_per_frame = (sbv_speed_ctrl->motor_right.motor->encoder_shift) -
                                 (sbv_speed_ctrl->motor_left.motor->encoder_shift);
    /* Update PID control */
    sbv_pid_update_output(&(sbv_speed_ctrl->steering_pid), wheel_diff_pulse_per_frame);
}

void
sbv_control_robot_speed_twist_update(sbv_control_robot_speed_t *sbv_speed_ctrl)
{
    float left_motor_output, right_motor_output;

    if(!sbv_speed_ctrl)
        return;

    sbv_control_robot_speed_update(sbv_speed_ctrl);
    sbv_control_robot_twist_update(sbv_speed_ctrl);

    left_motor_output = sbv_speed_ctrl->motor_left.motor_pid.output - sbv_speed_ctrl->steering_pid.output;
    right_motor_output = sbv_speed_ctrl->motor_right.motor_pid.output + sbv_speed_ctrl->steering_pid.output;

    sbv_motor_output_update((sbv_speed_ctrl->motor_left.motor), left_motor_output);
    sbv_motor_output_update((sbv_speed_ctrl->motor_right.motor), right_motor_output);
}