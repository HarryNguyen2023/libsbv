#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_can.h"

sbv_can_hw_cb_t sbv_can_hw_cb = {
#ifdef STM32F1xx
    .sbv_can_init       = sbv_can_stm32f1xx_init,
    .sbv_can_send_data  = sbv_can_stm32f1xx_send_data,
    .sbv_can_rcv_data   = sbv_can_stm32f1xx_rcv_data,
    .sbv_can_std_id_get = sbv_can_stm32f1xx_std_id_get,
#elif defined ESP32xx_IDF
    .sbv_can_init       = sbv_can_esp32s3_init,
    .sbv_can_send_data  = sbv_can_esp32s3_send_data,
    .sbv_can_rcv_data   = sbv_can_esp32s3_rcv_data,
    .sbv_can_std_id_get = sbv_can_esp32s3_std_id_get,
#endif /* STM32F1xx */
};

void
sbv_can_init(sbv_can_instance_t *can_instance, void *can_handle)
{
    if (sbv_can_hw_cb.sbv_can_init)
        (sbv_can_hw_cb.sbv_can_init) (can_instance, can_handle);
}

int
sbv_can_send_data(sbv_can_instance_t *can_instance,
                  sbv_can_msg_type_t msg_type,
                  uint8_t *data, uint16_t length)
{
    if (sbv_can_hw_cb.sbv_can_send_data)
        return (sbv_can_hw_cb.sbv_can_send_data) (can_instance, msg_type, data, length);

    return 0;
}

uint16_t
sbv_can_rcv_data(sbv_can_instance_t *can_instance,
                 uint8_t *rcv_buffer, uint16_t buffer_length,
                 uint16_t rcv_timeout_ms)
{
    if (sbv_can_hw_cb.sbv_can_rcv_data)
        return (sbv_can_hw_cb.sbv_can_rcv_data) (can_instance, rcv_buffer,
                                                 buffer_length, rcv_timeout_ms);
    return 0;
}