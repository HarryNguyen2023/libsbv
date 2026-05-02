#ifndef SBV_CONTROL_SPEED_H
#define SBV_CONTROL_SPEED_H

#include "sbv.h"

#define RPM_TO_RPS(RPS)     ((RPS) / 60.0)
#define ROBOT_SPEED_TO_MOTOR_SPEED(S,W)   \
    (((S) * 1000) / (SBV_PI * (W)))

typedef struct sbv_control_motor_speed_t
{
    sbv_motor_type_t    name;
    sbv_motor_t*        motor;
    sbv_pid_t           motor_pid;
} sbv_control_motor_speed_t;

typedef struct sbv_control_robot_speed_t
{
    float speed;
    float twist;

    float wheel_diameter;
    float wheel_distance;

    sbv_control_motor_speed_t motor_left;
    sbv_control_motor_speed_t motor_right;

    sbv_pid_t steering_pid;
} sbv_control_robot_speed_t;

void
sbv_control_robot_speed_init(sbv_control_robot_speed_t *sbv_speed_ctrl, float wheel_diameter, float wheel_distance);
void
sbv_control_robot_set_speed_target(sbv_control_robot_speed_t *sbv_speed_ctrl, float speed);
void
sbv_control_robot_set_twist_target(sbv_control_robot_speed_t *sbv_speed_ctrl, float twsit);
void
sbv_control_robot_speed_update_test(sbv_control_robot_speed_t *sbv_speed_ctrl);
void
sbv_control_robot_speed_update(sbv_control_robot_speed_t *sbv_speed_ctrl);
void
sbv_control_robot_twist_update_test(sbv_control_robot_speed_t *sbv_speed_ctrl);
void
sbv_control_robot_twist_update(sbv_control_robot_speed_t *sbv_speed_ctrl);

void
sbv_control_robot_set_target(sbv_control_robot_speed_t *sbv_speed_ctrl, float speed, float twist);
void
sbv_control_robot_speed_twist_update(sbv_control_robot_speed_t *sbv_speed_ctrl);
#endif /*SBV_CONTROL_SPEED_H*/