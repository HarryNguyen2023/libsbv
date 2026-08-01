#ifndef SBV_CONTROL_BALANCE_H
#define SBV_CONTROL_BALANCE_H

#include "sbv.h"
#include "sbv_imu.h"
#include "sbv_control_speed.h"

typedef struct sbv_control_balance_t
{
    sbv_imu_instance_t          *imu;
    sbv_pid_t                   balance_pid;
    sbv_control_robot_speed_t   sbv_control_speed;
    sbv_rtos_mutex_t            mu;
} sbv_control_balance_t;

void
sbv_control_balance_init(sbv_control_balance_t *sbv_ctrl_balance, sbv_imu_instance_t *imu_instance,
                         sbv_i2c_instance_t *i2c_instance, sbv_i2c_handle_t *i2c_handle);
void
sbv_control_balance_update(sbv_control_balance_t *sbv_ctrl_balance);
void
sbv_control_balance_update_speed_twist_control (sbv_control_balance_t *sbv_ctrl_balance);
uint16_t
sbv_control_balance_get_balance_sampling_time_ms (sbv_control_balance_t *sbv_ctrl_balance);
uint16_t
sbv_control_balance_get_speed_sampling_time_ms (sbv_control_balance_t *sbv_ctrl_balance);
void
sbv_control_balance_set_balance_pid_gain (sbv_control_balance_t *sbv_ctrl_balance,
                                          float Kp, float Ki, float Kd);
void
sbv_control_balance_set_steering_pid_gain (sbv_control_balance_t *sbv_ctrl_balance,
                                          float Kp, float Ki, float Kd);
void
sbv_control_balance_set_speed_pid_gain (sbv_control_balance_t *sbv_ctrl_balance,
                                        sbv_motor_type_t motor, float Kp, float Ki, float Kd);
void
sbv_control_balance_set_balance_control_target (sbv_control_balance_t *sbv_ctrl_balance, float target);
void
sbv_control_balance_set_robot_twist_target (sbv_control_balance_t *sbv_ctrl_balance, float target);
void
sbv_control_balance_set_robot_speed_target (sbv_control_balance_t *sbv_ctrl_balance, float target);
void
sbv_control_balance_set_robot_speed_twist_target (sbv_control_balance_t *sbv_ctrl_balance,
                                                 float speed, float twist);
void
sbv_control_balance_set_motor_speed_target (sbv_control_balance_t *sbv_ctrl_balance,
                                            sbv_motor_type_t motor, float target);
void
sbv_control_balance_reset_balance_pid (sbv_control_balance_t *sbv_ctrl_balance);
void
sbv_control_balance_reset_steering_pid (sbv_control_balance_t *sbv_ctrl_balance);
void
sbv_control_balance_reset_speed_pid (sbv_control_balance_t *sbv_ctrl_balance,
                                     sbv_motor_type_t motor);
void
sbv_control_balance_reset_encoder (sbv_control_balance_t *sbv_ctrl_balance);
void
sbv_control_balance_get_encoder (sbv_control_balance_t *sbv_ctrl_balance,
                                char *data_buffer, const uint16_t len);
void
sbv_control_balance_get_imu (sbv_control_balance_t *sbv_ctrl_balance,
                            char *data_buffer, const uint16_t len);
void
sbv_control_balance_get_pid (sbv_control_balance_t *sbv_ctrl_balance, sbv_pid_t *pid,
                            char *data_buffer, const uint16_t len);
#endif /*SBV_CONTROL_BALANCE_H*/