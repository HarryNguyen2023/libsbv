#include "sbv.h"
#include "sbv_pid.h"

void
sbv_pid_init (sbv_pid_t *pid, float max_output, float min_output,
              float Kp, float Ki, float Kd, uint16_t sampling_time_ms)
{
    if(! pid)
        return;

    if (max_output <= min_output)
    {
        /* Log */
        return;
    }

    pid->target             = 0;
    pid->feedback           = 0;
    pid->error              = 0;
    pid->prev_error         = 0;
    pid->Kp                 = Kp;
    pid->Ki                 = Ki;
    pid->Kd                 = Kd;
    pid->I_error            = 0;
    pid->output             = 0;
    pid->max_output         = max_output;
    pid->min_output         = min_output;
    pid->sampling_time_ms   = sampling_time_ms;
}

void
sbv_pid_set_target (sbv_pid_t *pid, float target)
{
    if(! pid)
        return;

    pid->target = target;
}

void
sbv_pid_set_gain (sbv_pid_t *pid, float Kp, float Ki, float Kd)
{
    if(! pid)
        return;

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void
sbv_pid_reset (sbv_pid_t *pid)
{
    if(! pid)
        return;

    pid->feedback   = 0;
    pid->error      = 0;
    pid->prev_error = 0;
    pid->I_error    = 0;
    pid->output     = 0;
}

static void
sbv_pid_anti_integral_windup (sbv_pid_t *pid, float pid_prop)
{
    float lim_max_int, lim_min_int;

    if(! pid)
        return;

    /* Get the maximum and minimum limit of integral error */
    if(pid_prop < pid->max_output)
        lim_max_int = pid->max_output - pid_prop;
    else 
        lim_max_int = 0;

    if(pid_prop > pid->min_output)
        lim_min_int = pid->min_output - pid_prop;
    else
        lim_min_int = 0;

    /* Limit the actual integral error */
    if(pid->I_error > lim_max_int)
        pid->I_error = lim_max_int;
    else if(pid->I_error < lim_min_int)
        pid->I_error = lim_min_int;
}

static inline void
sbv_pid_output_limit (sbv_pid_t *pid)
{
    if(! pid)
        return;

    if(pid->output > pid->max_output)
        pid->output = pid->max_output;
    else if(pid->output < pid->min_output)
        pid->output = pid->min_output;
}

void
sbv_pid_update_output (sbv_pid_t *pid, float feedback)
{
    float pid_prop, pid_drev;

    if(! pid)
        return;

    pid->feedback   = feedback;
    pid->error      = pid->target - pid->feedback;
    pid_prop        = pid->Kp * pid->error;
    pid_drev        = pid->Kd * (pid->error - pid->prev_error);
    pid->I_error    += pid->Ki * pid->error;

    /* Perform anti integral windup */
    if(pid->Ki)
        sbv_pid_anti_integral_windup(pid, pid_prop + pid_drev);

    pid->output     = pid_prop + pid->I_error + pid_drev;
    sbv_pid_output_limit(pid);

    pid->prev_error = pid->error;
}