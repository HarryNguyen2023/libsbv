#include "sbv.h"
#include "sbv_log.h"
#include "sbv_gpio_esp32s3.h"
#ifdef ESP32xx_IDF

void
sbv_gpio_esp32s3_init (uint8_t *unsused, sbv_gpio_num_t gpio_num, sbv_gpio_mode_t gpio_mode)
{
    if (gpio_num >= SBV_GPIO_NUM_MAX) {
        LOG_ERROR ("Invalid GPIO num %u, max GPIO %u", gpio_num, SBV_GPIO_NUM_MAX);
        return;
    }

    gpio_set_direction (gpio_num, gpio_mode);
}

void
sbv_gpio_esp32s3_set_pin_level (uint8_t *unsused, sbv_gpio_num_t gpio_num, sbv_gpio_pin_state_t pin_state)
{
    if (gpio_num >= SBV_GPIO_NUM_MAX) {
        LOG_ERROR ("Invalid GPIO num %u, max GPIO %u", gpio_num, SBV_GPIO_NUM_MAX);
        return;
    }

    gpio_set_level (gpio_num, pin_state);
}

uint8_t
sbv_gpio_esp32s3_read_pin (uint8_t *unsused, sbv_gpio_num_t gpio_num)
{
    if (gpio_num >= SBV_GPIO_NUM_MAX) {
        LOG_ERROR ("Invalid GPIO num %u, max GPIO %u", gpio_num, SBV_GPIO_NUM_MAX);
        return 2;
    }

    return gpio_get_level (gpio_num);
}

void
sbv_gpio_esp32s3_toggle_pin (uint8_t *unsused, sbv_gpio_num_t gpio_num, uint16_t delay_ms)
{
    sbv_gpio_pin_state_t cur_pin_state;
    sbv_rtos_tick_type_t delay_tick = sbv_rtos_ms_to_tick(delay_ms);

    if (gpio_num >= SBV_GPIO_NUM_MAX) {
        LOG_ERROR ("Invalid GPIO num %u, max GPIO %u", gpio_num, SBV_GPIO_NUM_MAX);
        return;
    }

    cur_pin_state = sbv_gpio_esp32s3_read_pin (gpio_num);

    sbv_gpio_esp32s3_set_pin_level (gpio_num, (! cur_pin_state));
    sbv_rtos_task_delay (delay_tick);
}
#endif /* ESP32xx_IDF */