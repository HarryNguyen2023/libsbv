#ifndef SBV_MOTOR_H
#define SBV_MOTOR_H

#include "sbv.h"

#ifdef STM32F1xx
#include "stm32f1xx_hal_tim.h"

#define SBV_TIM_1   TIM1
#define SBV_TIM_2   TIM2
#define SBV_TIM_3   TIM3
#define SBV_TIM_4   TIM4

typedef TIM_TypeDef sbv_tim_type_def;

typedef enum sbv_motor_pwm_t
{
    SBV_PWM_CHANNEL_1,
    SBV_PWM_CHANNEL_2,
    SBV_PWM_CHANNEL_3,
    SBV_PWM_CHANNEL_4
}sbv_motor_pwm_t;
#endif /*STM32F1xx*/

typedef enum sbv_motor_type_t
{
    SBV_MOTOR_LEFT,
    SBV_MOTOR_RIGHT
} sbv_motor_type_t;

typedef enum sbv_motor_dir_t
{
    SBV_MOTOR_DIR_CW,
    SBV_MOTOR_DIR_CCW
} sbv_motor_dir_t;

typedef struct sbv_motor_t
{
    sbv_gpio_type_def*  motor_ports[2];
    sbv_gpio_num_t      motor_pin[2];

#ifdef STM32F1xx
    sbv_motor_pwm_t     pwm_channel;
    sbv_tim_type_def*   encoder_tim;
    sbv_tim_type_def*   pwm_tim;
#endif /* STM32F1xx */

    uint16_t            encoder_rev;
    sbv_motor_dir_t     dir;
    uint8_t             moving;
    uint16_t            pwm_output;

    uint16_t            max_pwm;
    uint16_t            max_speed_rpm;
    uint8_t             dead_band_dc;

    uint16_t            encoder_pos;
    uint16_t            prev_encoder_pos;
    uint16_t            encoder_shift;
} sbv_motor_t;

void
sbv_motor_init(sbv_motor_t *motor);
void
sbv_motor_get_speed(sbv_motor_t *motor);
uint16_t
sbv_motor_read_encoder(sbv_motor_t *motor);
void
sbv_motor_encoder_reset(sbv_motor_t *motor);
void
sbv_motor_output_update(sbv_motor_t *motor, int16_t duty_cycle);
void
sbv_motor_brake(sbv_motor_t *motor);
void
sbv_motor_move_cw(sbv_motor_t *motor);
void
sbv_motor_move_ccw(sbv_motor_t *motor);
#endif /*SBV_MOTOR_H*/