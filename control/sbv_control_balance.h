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

#endif /*SBV_CONTROL_BALANCE_H*/