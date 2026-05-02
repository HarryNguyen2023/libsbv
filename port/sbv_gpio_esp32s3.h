#ifndef __SBV_GPIO_ESP32S3_H__
#define __SBV_GPIO_ESP32S3_H__

#include "sbv.h"

#ifdef ESP32xx_IDF
#include "driver/gpio.h"

#define SBV_GPIO_BUILT_IN_LED       GPIO_NUM_13
#define SBV_GPIO_BUILT_IN_LED_TYPE  NULL

typedef uint32_t   sbv_gpio_type_def;

typedef enum sbv_gpio_num_t
{
    SBV_GPIO_NUM_0,
    SBV_GPIO_NUM_1,
    SBV_GPIO_NUM_2,
    SBV_GPIO_NUM_3,
    SBV_GPIO_NUM_4,
    SBV_GPIO_NUM_5,
    SBV_GPIO_NUM_6,
    SBV_GPIO_NUM_7,
    SBV_GPIO_NUM_8,
    SBV_GPIO_NUM_9,
    SBV_GPIO_NUM_10,
    SBV_GPIO_NUM_11,
    SBV_GPIO_NUM_12,
    SBV_GPIO_NUM_13,
    SBV_GPIO_NUM_14,
    SBV_GPIO_NUM_15,
    SBV_GPIO_NUM_16,
    SBV_GPIO_NUM_17,
    SBV_GPIO_NUM_18,
    SBV_GPIO_NUM_19,
    SBV_GPIO_NUM_20,
    SBV_GPIO_NUM_21,
    SBV_GPIO_NUM_22,
    SBV_GPIO_NUM_23,
    SBV_GPIO_NUM_24,
    SBV_GPIO_NUM_25,
    SBV_GPIO_NUM_26,
    SBV_GPIO_NUM_27,
    SBV_GPIO_NUM_28,
    SBV_GPIO_NUM_29,
    SBV_GPIO_NUM_30,
    SBV_GPIO_NUM_31,
    SBV_GPIO_NUM_32,
    SBV_GPIO_NUM_33,
    SBV_GPIO_NUM_34,
    SBV_GPIO_NUM_35,
    SBV_GPIO_NUM_36,
    SBV_GPIO_NUM_37,
    SBV_GPIO_NUM_38,
    SBV_GPIO_NUM_39,
    SBV_GPIO_NUM_MAX,
} sbv_gpio_num_t;

typedef enum sbv_gpio_mode_t
{  
    SBV_GPIO_MODE_INPUT     = GPIO_MODE_INPUT,
    SBV_GPIO_MODE_OUTPUT    = GPIO_MODE_OUTPUT,
} sbv_gpio_mode_t;

typedef enum sbv_gpio_pin_state_t
{
    SBV_GPIO_PIN_HIGH       = 1u,
    SBV_GPIO_PIN_LOW        = 0u,
} sbv_gpio_pin_state_t;

void
sbv_gpio_esp32s3_init (uint8_t *unsused, sbv_gpio_num_t gpio_num, sbv_gpio_mode_t gpio_mode);
void
sbv_gpio_esp32s3_set_pin_level (uint8_t *unsused, sbv_gpio_num_t gpio_num, sbv_gpio_pin_state_t pin_state);
uint8_t
sbv_gpio_esp32s3_read_pin (uint8_t *unsused, sbv_gpio_num_t gpio_num);
void
sbv_gpio_esp32s3_toggle_pin (uint8_t *unsused, sbv_gpio_num_t gpio_num, uint16_t delay_ms);

#endif /* ESP32xx_IDF */
#endif /* __SBV_GPIO_ESP32S3_H__ */