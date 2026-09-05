#ifndef __SBV_CONTROL_CFG_H__
#define __SBV_CONTROL_CFG_H__

#define SBV_ROBOT_WHEEL_DIAMETER     62
#define SBV_ROBOT_WHEEL_DISTANCE     200

#define MOTOR_LEFT_PID_KP           (44.2f)
#define MOTOR_LEFT_PID_KI           (1.39f)

#define MOTOR_RIGHT_PID_KP          (44.2f)
#define MOTOR_RIGHT_PID_KI          (1.31f)

#define MOTOR_PID_SAMPLING_TIME_MS  20

#define STERRING_PID_KP             100
#define STEERING_PID_KD             (0.6f)
#define STEERING_PID_MIN_OUTPUT     (-200)
#define STEERING_PID_MAX_OUTPUT     200
#define STEERING_PID_SAMPLING_TIME  20

#define BALANCE_PID_KP              100
#define BALANCE_PID_KD              (0.6f)
#define BALANCE_PID_MIN_OUTPUT      (-200)
#define BALANCE_PID_MAX_OUTPUT      200
#define BALANCE_PID_SAMPLING_TIME   80

#endif /* __SBV_CONTROL_CFG_H__ */