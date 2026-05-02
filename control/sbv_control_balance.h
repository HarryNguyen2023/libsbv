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
sbv_control_balance_init(sbv_control_balance_t *sbv_ctrl_balance, sbv_i2c_handle_t *i2c_handle);
void
sbv_control_balance_update(sbv_control_balance_t *sbv_ctrl_balance);

#endif /*SBV_CONTROL_BALANCE_H*/