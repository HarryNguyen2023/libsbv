#include "sbv.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"
#include "sbv_pid.h"
#include "sbv_control_cfg.h"

sbv_motor_t motor_left = 
{
#ifdef STM32F1xx
    .motor_ports        = {SBV_GPIO_B, SBV_GPIO_B},
#else
    .motor_ports        = {NULL, NULL},
#endif /* STM32F1xx */
    .motor_pin          = {SBV_GPIO_NUM_4, SBV_GPIO_NUM_5},

#ifdef STM32F1xx
    .pwm_channel        = SBV_PWM_CHANNEL_1,
    .encoder_tim        = SBV_TIM_2,
    .pwm_tim            = SBV_TIM_4,
#endif /* STM32F1xx */

    .encoder_rev        = 374,

    .max_pwm            = 500,
    .max_speed_rpm      = 250,
    .dead_band_dc       = 20,
};

sbv_motor_t motor_right = 
{
#ifdef STM32F1xx
    .motor_ports        = {SBV_GPIO_B, SBV_GPIO_A},
#else
    .motor_ports        = {NULL, NULL},
#endif /* STM32F1xx */
    .motor_pin          = {SBV_GPIO_NUM_3, SBV_GPIO_NUM_15},

#ifdef STM32F1xx
    .pwm_channel        = SBV_PWM_CHANNEL_2,
    .encoder_tim        = SBV_TIM_3,
    .pwm_tim            = SBV_TIM_4,
#endif /* STM32F1xx */

    .encoder_rev        = 374,

    .max_pwm            = 500,
    .max_speed_rpm      = 250,
    .dead_band_dc       = 20,
};