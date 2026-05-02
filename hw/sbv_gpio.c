#include "sbv.h"
#include "sbv_gpio.h"
#include "sbv_rtos.h"

struct sbv_gpio_hw_cb sbv_gpio_hw =
{
#ifdef STM32F1xx
    .sbv_gpio_init       = sbv_gpio_stm32f1xx_init,
    .sbv_gpio_set_pin    = sbv_gpio_stm32f1xx_set_pin_level,
    .sbv_gpio_read_pin   = sbv_gpio_stm32f1xx_read_pin,
    .sbv_gpio_toggle_pin = sbv_gpio_stm32f1xx_toggle_pin
#elif defined ESP32xx_IDF
    .sbv_gpio_init       = sbv_gpio_esp32s3_init,
    .sbv_gpio_read_pin   = sbv_gpio_esp32s3_set_pin_level,
    .sbv_gpio_read_pin   = sbv_gpio_esp32s3_read_pin,
    .sbv_gpio_toggle_pin = sbv_gpio_esp32s3_toggle_pin
#endif /* STM32F1xx */
};

void sbv_gpio_init (void *gpio_type, sbv_gpio_num_t gpio_num, sbv_gpio_mode_t gpio_mode)
{
    if (sbv_gpio_hw.sbv_gpio_init)
        (sbv_gpio_hw.sbv_gpio_init) (gpio_type, gpio_num, gpio_mode);
}

void sbv_gpio_set_pin (void *gpio_type, sbv_gpio_num_t gpio_num, sbv_gpio_pin_state_t pin_state)
{
    if (sbv_gpio_hw.sbv_gpio_set_pin)
        (sbv_gpio_hw.sbv_gpio_set_pin) (gpio_type, gpio_num, pin_state);
}

uint8_t sbv_gpio_read_pin (void *gpio_type, sbv_gpio_num_t gpio_num)
{
    if (sbv_gpio_hw.sbv_gpio_read_pin)
        return (sbv_gpio_hw.sbv_gpio_read_pin) (gpio_type, gpio_num);

    return 2;
}

void sbv_gpio_toggle_pin (void *gpio_type, sbv_gpio_num_t gpio_num, uint16_t delay_ms)
{
    if (sbv_gpio_hw.sbv_gpio_toggle_pin)
        (sbv_gpio_hw.sbv_gpio_toggle_pin) (gpio_type, gpio_num, delay_ms);
}