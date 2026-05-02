#ifndef SBV_GPIO_H
#define SBV_GPIO_H

#include "sbv.h"

#ifdef STM32F1xx
#include "sbv_gpio_stm32f1xx.h"
#elif defined ESP32xx_IDF
#include "sbv_gpio_esp32s3.h"
#endif /* STM32F1xx */

struct sbv_gpio_hw_cb
{
    void (*sbv_gpio_init) (void *, sbv_gpio_num_t , sbv_gpio_mode_t );
    void (*sbv_gpio_set_pin) (void *, sbv_gpio_num_t, sbv_gpio_pin_state_t);
    uint8_t (*sbv_gpio_read_pin) (void *, sbv_gpio_num_t);
    void (*sbv_gpio_toggle_pin) (void *, sbv_gpio_num_t, uint16_t);
};

void sbv_gpio_init (void *, sbv_gpio_num_t , sbv_gpio_mode_t );
void sbv_gpio_set_pin (void *, sbv_gpio_num_t, sbv_gpio_pin_state_t);
uint8_t sbv_gpio_read_pin (void *, sbv_gpio_num_t);
void sbv_gpio_toggle_pin (void *, sbv_gpio_num_t, uint16_t);

#endif /*SBV_GPIO_H*/