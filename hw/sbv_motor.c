#include "math.h"
#include "sbv.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"

#define SBV_MAX_ENCODER_VALUE   (65535 / 4)
#define SBV_ENCODER_ROLLOVER_THRESHOLD   16000

void
sbv_motor_init(sbv_motor_t *motor)
{
    u_int16_t tim_channel = 0;

    if(! motor)
        return;

    sbv_gpio_init(motor->motor_ports[0], motor->motor_pin[0], SBV_GPIO_MODE_OUTPUT);
    sbv_gpio_init(motor->motor_ports[1], motor->motor_pin[1], SBV_GPIO_MODE_OUTPUT);

#ifdef STM32F1xx
    switch (motor->pwm_channel)
    {
    case SBV_PWM_CHANNEL_1:
        tim_channel = TIM_CHANNEL_1;
        break;
    case SBV_PWM_CHANNEL_2:
        tim_channel = TIM_CHANNEL_2;
        break;
    case SBV_PWM_CHANNEL_3:
        tim_channel = TIM_CHANNEL_3;
        break;
    case SBV_PWM_CHANNEL_4:
        tim_channel = TIM_CHANNEL_4;
        break;
    default:
        break;
    }
    HAL_TIM_PWM_Start (motor->pwm_tim, tim_channel);
    HAL_TIM_Encoder_Start (motor->encoder_tim, TIM_CHANNEL_ALL);
#endif /* STM32F1xx */
    
    motor->pwm_output       = 0;

    motor->dir              = SBV_MOTOR_DIR_CW;
    motor->moving           = SBV_FALSE;

    motor->encoder_pos      = 0;
    motor->prev_encoder_pos = 0;
    motor->encoder_shift    = 0;
}

uint16_t
sbv_motor_read_encoder(sbv_motor_t *motor)
{
    if(! motor)
        return 0;

#ifdef STM32F1xx
    return (motor->encoder_tim->CNT >> 2);
#endif /*STM32F1xx*/
}

void
sbv_motor_encoder_reset(sbv_motor_t *motor)
{
    if(! motor)
        return;

#ifdef STM32F1xx
    motor->encoder_tim->CNT = 0;
#endif /*STM32F1xx*/
}

void
sbv_motor_get_speed(sbv_motor_t *motor)
{
    
    if(! motor)
        return;

    motor->encoder_pos = sbv_motor_read_encoder(motor);

    /* Avoid counter overflow */
    if(motor->dir == SBV_MOTOR_DIR_CW && (motor->encoder_pos < motor->prev_encoder_pos)
       && (motor->prev_encoder_pos - motor->encoder_pos > SBV_ENCODER_ROLLOVER_THRESHOLD))
    {
        motor->encoder_shift = SBV_MAX_ENCODER_VALUE - motor->prev_encoder_pos;
        motor->encoder_shift += motor->encoder_pos;
    }
    else if(motor->dir == SBV_MOTOR_DIR_CCW && (motor->encoder_pos > motor->prev_encoder_pos)
            && (motor->encoder_pos - motor->prev_encoder_pos > SBV_ENCODER_ROLLOVER_THRESHOLD))
    {
        motor->encoder_shift = motor->encoder_pos - SBV_MAX_ENCODER_VALUE;
        motor->encoder_shift -= motor->prev_encoder_pos;
    }
    else
    {
        motor->encoder_shift = motor->encoder_pos - motor->prev_encoder_pos;
    }

    motor->prev_encoder_pos = motor->encoder_pos;
}

static void
sbv_motor_pwm_update(sbv_motor_t *motor, uint16_t pwm_duty_cycle)
{
    if(! motor)
        return;

    motor->pwm_output = pwm_duty_cycle;

#ifdef STM32F1xx
    switch (motor->pwm_channel)
    {
    case SBV_PWM_CHANNEL_1:
        motor->pwm_tim->CCR1 = pwm_duty_cycle;
        break;
    case SBV_PWM_CHANNEL_2:
        motor->pwm_tim->CCR2 = pwm_duty_cycle;
        break;
    case SBV_PWM_CHANNEL_3:
        motor->pwm_tim->CCR3 = pwm_duty_cycle;
        break;
    case SBV_PWM_CHANNEL_4:
        motor->pwm_tim->CCR4 = pwm_duty_cycle;
        break;
    default:
        break;
    }
#endif /*STM32F1xx*/
}

void
sbv_motor_output_update(sbv_motor_t *motor, int16_t duty_cycle)
{
    int16_t duty_cycle_output;

    if(! motor)
        return;

    if(duty_cycle == 0
        || abs(duty_cycle) < motor->dead_band_dc)
    {
        sbv_motor_brake(motor);
        return;
    }
    else if(duty_cycle > 0)
    {
        motor->dir = SBV_MOTOR_DIR_CW;
        sbv_motor_move_cw(motor);
    }
    else if(duty_cycle < 0)
    {
        motor->dir = SBV_MOTOR_DIR_CCW;
        sbv_motor_move_ccw(motor);
    }

    duty_cycle_output = (duty_cycle > 0) ? duty_cycle : -(duty_cycle);
    duty_cycle_output = (duty_cycle_output < motor->max_pwm) \
                            ? duty_cycle_output : motor->max_pwm;
    sbv_motor_pwm_update(motor, duty_cycle_output);
}

void
sbv_motor_brake(sbv_motor_t *motor)
{
    if(! motor)
        return;

    sbv_motor_pwm_update(motor, 0);

    sbv_gpio_set_pin(motor->motor_ports[0], motor->motor_pin[0], SBV_GPIO_PIN_LOW);
    sbv_gpio_set_pin(motor->motor_ports[1], motor->motor_pin[1], SBV_GPIO_PIN_LOW);

    motor->moving = SBV_FALSE;
}

void
sbv_motor_move_cw(sbv_motor_t *motor)
{
    if(! motor)
        return;

    sbv_gpio_set_pin(motor->motor_ports[0], motor->motor_pin[0], SBV_GPIO_PIN_HIGH);
    sbv_gpio_set_pin(motor->motor_ports[1], motor->motor_pin[1], SBV_GPIO_PIN_LOW);

    motor->moving = SBV_TRUE;
}

void
sbv_motor_move_ccw(sbv_motor_t *motor)
{
    if(! motor)
        return;

    sbv_gpio_set_pin(motor->motor_ports[0], motor->motor_pin[0], SBV_GPIO_PIN_LOW);
    sbv_gpio_set_pin(motor->motor_ports[1], motor->motor_pin[1], SBV_GPIO_PIN_HIGH);

    motor->moving = SBV_TRUE;
}