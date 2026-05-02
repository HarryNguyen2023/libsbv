#ifndef __SBV_GPIO_STM32F1XX_H__
#define __SBV_GPIO_STM32F1XX_H__

#include "sbv.h"

#ifdef STM32F1xx

#define SBV_GPIO_BUILT_IN_LED       GPIO_PIN_13
#define SBV_GPIO_BUILT_IN_LED_TYPE  GPIOC

#define SBV_GPIO_A GPIOA
#define SBV_GPIO_B GPIOB
#define SBV_GPIO_C GPIOC
#define SBV_GPIO_D GPIOD
#define SBV_GPIO_E GPIOE

#define SBV_GPIO_CLOCK_A_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE
#define SBV_GPIO_CLOCK_B_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
#define SBV_GPIO_CLOCK_C_ENABLE __HAL_RCC_GPIOC_CLK_ENABLE
#define SBV_GPIO_CLOCK_D_ENABLE __HAL_RCC_GPIOD_CLK_ENABLE
#define SBV_GPIO_CLOCK_E_ENABLE __HAL_RCC_GPIOE_CLK_ENABLE

typedef enum sbv_gpio_num_t
{
    SBV_GPIO_NUM_0  = GPIO_PIN_0,
    SBV_GPIO_NUM_1  = GPIO_PIN_1,
    SBV_GPIO_NUM_2  = GPIO_PIN_2,
    SBV_GPIO_NUM_3  = GPIO_PIN_3,
    SBV_GPIO_NUM_4  = GPIO_PIN_4,
    SBV_GPIO_NUM_5  = GPIO_PIN_5,
    SBV_GPIO_NUM_6  = GPIO_PIN_6,
    SBV_GPIO_NUM_7  = GPIO_PIN_7,
    SBV_GPIO_NUM_8  = GPIO_PIN_8,
    SBV_GPIO_NUM_9  = GPIO_PIN_9,
    SBV_GPIO_NUM_10 = GPIO_PIN_10,
    SBV_GPIO_NUM_11 = GPIO_PIN_11,
    SBV_GPIO_NUM_12 = GPIO_PIN_12,
    SBV_GPIO_NUM_13 = GPIO_PIN_13,
    SBV_GPIO_NUM_14 = GPIO_PIN_14,
    SBV_GPIO_NUM_15 = GPIO_PIN_15,
    SBV_GPIO_NUM_MAX = (SBV_GPIO_NUM_15 << 1)
} sbv_gpio_num_t;

typedef enum sbv_gpio_mode_t
{
    SBV_GPIO_MODE_INPUT     = GPIO_MODE_INPUT,
    SBV_GPIO_MODE_OUTPUT    = GPIO_MODE_OUTPUT_PP,
} sbv_gpio_mode_t;

typedef enum sbv_gpio_pin_state_t
{
    SBV_GPIO_PIN_HIGH       = GPIO_PIN_SET,
    SBV_GPIO_PIN_LOW        = GPIO_PIN_RESET,
} sbv_gpio_pin_state_t;

typedef GPIO_TypeDef        sbv_gpio_type_def;
typedef GPIO_InitTypeDef    sbv_gpio_init_type_def;

typedef enum sbv_gpio_pull_t
{
    SBV_GPIO_NO_PULL    = GPIO_NOPULL,
    SBV_GPIO_PULL_UP    = GPIO_PULLUP,
    SBV_GPIO_PULL_DOWN  = GPIO_PULLDOWN,
} sbv_gpio_pull_t;

typedef enum sbv_gpio_speed_t
{
    SBV_GPIO_SPEED_LOW      = GPIO_SPEED_FREQ_LOW,
    SBV_GPIO_SPEED_MEDIUM   = GPIO_SPEED_FREQ_MEDIUM,
    SBV_GPIO_SPEED_HIGH     = GPIO_SPEED_FREQ_HIGH,
} sbv_gpio_speed_t;

void
sbv_gpio_stm32f1xx_init (sbv_gpio_type_def *gpio_type, sbv_gpio_num_t gpio_num, sbv_gpio_mode_t gpio_mode);
void
sbv_gpio_stm32f1xx_set_pin_level (sbv_gpio_type_def *gpio_type, sbv_gpio_num_t gpio_num, sbv_gpio_pin_state_t pin_state);
uint8_t
sbv_gpio_stm32f1xx_read_pin (sbv_gpio_type_def *gpio_type, sbv_gpio_num_t gpio_num);
void
sbv_gpio_stm32f1xx_toggle_pin (sbv_gpio_type_def *gpio_type, sbv_gpio_num_t gpio_num, uint16_t delay_ms);

#endif /* STM32F1xx */
#endif /* __SBV_GPIO_STM32F1XX_H__ */