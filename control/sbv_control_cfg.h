#ifndef __SBV_CONTROL_CFG_H__
#define __SBV_CONTROL_CFG_H__

#define MOTOR_LEFT_PID_KP           44.2
#define MOTOR_LEFT_PID_KI           1.39

#define MOTOR_RIGHT_PID_KP          44.2
#define MOTOR_RIGHT_PID_KI          1.31

#define MOTOR_PID_SAMPLING_TIME_MS  20

#define STERRING_PID_KP             100
#define STEERING_PID_KD             0.6
#define STEERING_PID_MAX_OUTPUT     500
#define STEERING_PID_SAMPLING_TIME  20

#define BALANCE_PID_KP              100
#define BALANCE_PID_KD              0.6
#define BALANCE_PID_MAX_OUTPUT      500
#define BALANCE_PID_SAMPLING_TIME   80

#endif /* __SBV_CONTROL_CFG_H__ */