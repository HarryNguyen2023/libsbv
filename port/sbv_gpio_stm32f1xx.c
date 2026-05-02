#include "sbv_rtos.h"
#include "sbv_gpio_stm32f1xx.h"
#ifdef STM32F1xx
void
sbv_gpio_stm32f1xx_init (sbv_gpio_type_def *gpio_type, sbv_gpio_num_t gpio_num, sbv_gpio_mode_t gpio_mode)
{
    sbv_gpio_init_type_def gpio_init = {0};

    if (! gpio_type || gpio_num >= SBV_GPIO_NUM_MAX)
        return;

    if(gpio_type == SBV_GPIO_A)
        SBV_GPIO_CLOCK_A_ENABLE();
    else if(gpio_type == SBV_GPIO_B)
        SBV_GPIO_CLOCK_B_ENABLE();
    else if(gpio_type == SBV_GPIO_C)
        SBV_GPIO_CLOCK_C_ENABLE();
    else if(gpio_type == SBV_GPIO_D)
        SBV_GPIO_CLOCK_D_ENABLE();
    else if(gpio_type == SBV_GPIO_E)
        SBV_GPIO_CLOCK_E_ENABLE();

    sbv_gpio_stm32f1xx_set_pin_level(gpio_type, gpio_num, SBV_GPIO_PIN_LOW);

    gpio_init.Pin   = gpio_num;
    gpio_init.Mode  = gpio_mode;
    if(gpio_mode == SBV_GPIO_MODE_INPUT)
        gpio_init.Pull = SBV_GPIO_PULL_UP;
    else if(gpio_mode == SBV_GPIO_MODE_OUTPUT)
        gpio_init.Pull = SBV_GPIO_NO_PULL;
    gpio_init.Speed = SBV_GPIO_SPEED_LOW;

    HAL_GPIO_Init(gpio_type, &gpio_init);
}

void
sbv_gpio_stm32f1xx_set_pin_level (sbv_gpio_type_def *gpio_type, sbv_gpio_num_t gpio_num, sbv_gpio_pin_state_t pin_state)
{
    if (! gpio_type || gpio_num >= SBV_GPIO_NUM_MAX)
        return;

    HAL_GPIO_WritePin(gpio_type, gpio_num, pin_state);
}

uint8_t
sbv_gpio_stm32f1xx_read_pin (sbv_gpio_type_def *gpio_type, sbv_gpio_num_t gpio_num)
{
    if (! gpio_type || gpio_num >= SBV_GPIO_NUM_MAX)
        return 2;

    return HAL_GPIO_ReadPin (gpio_type, gpio_num);
}

void
sbv_gpio_stm32f1xx_toggle_pin(sbv_gpio_type_def *gpio_type, sbv_gpio_num_t gpio_num, uint16_t delay_ms)
{
    sbv_rtos_tick_type_t delay_tick = sbv_rtos_ms_to_tick(delay_ms);

    if (! gpio_type || gpio_num >= SBV_GPIO_NUM_MAX)
        return;

    HAL_GPIO_TogglePin (gpio_type, gpio_num);
    if (delay_ms)
        sbv_rtos_task_delay (delay_tick);
}
#endif /* STM32F1xx */