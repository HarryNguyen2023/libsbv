#ifndef SBV_PID_H
#define SBV_PID_H

#include "sbv.h"

typedef struct sbv_pid_t
{
    float       target;
    float       feedback;
    float       error;
    float       prev_error;
    float       Kp;
    float       Ki;
    float       Kd;
    float       I_error;
    float       output;
    float       max_output;
    float       min_output;
    uint16_t    sampling_time_ms;
} sbv_pid_t;

void
sbv_pid_init (sbv_pid_t *pid, float max_output, float min_output,
              float Kp, float Ki, float Kd, uint16_t sampling_time_ms);
void
sbv_pid_set_target (sbv_pid_t *pid, float target);
void
sbv_pid_set_gain (sbv_pid_t *pid, float Kp, float Ki, float Kd);
void
sbv_pid_reset (sbv_pid_t *pid);
void
sbv_pid_update_output (sbv_pid_t *pid, float feedback);
#endif /*SBV_PID_H*/