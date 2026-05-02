#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_can.h"
#include "sbv_ota.h"
#include "sbv_ota_msg.h"

sbv_rtos_mutex_t SBV_CAN_RX_BUFFER_MUTEX;
sbv_rtos_mutex_t SBV_CAN_TX_BUFFER_MUTEX;

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
sbv_can_init(void *can_handle)
{
    if (sbv_can_hw_cb.sbv_can_init)
        (sbv_can_hw_cb.sbv_can_init) (can_handle);
}

int
sbv_can_send_data(sbv_can_msg_type_t msg_type, uint8_t *data, uint8_t length)
{
    if (sbv_can_hw_cb.sbv_can_send_data)
        return (sbv_can_hw_cb.sbv_can_send_data) (msg_type, data, length);

    return 0;
}

uint8_t *
sbv_can_rcv_data (uint8_t *length, uint16_t *std_id)
{
    if (sbv_can_hw_cb.sbv_can_rcv_data)
        return (sbv_can_hw_cb.sbv_can_rcv_data) (length, std_id);

    *length = 0;
    *std_id = 0;
    return NULL;
}

void
sbv_can_pkt_process(uint8_t *data, uint8_t length, uint16_t std_id)
{
    if(! data || ! (sbv_can_hw_cb.sbv_can_std_id_get))
        return;

    if(std_id == (sbv_can_hw_cb.sbv_can_std_id_get) (SBV_CAN_STD_ID_OTA))
    {
        sbv_ota_msg_rx_handle(data, length);
    }
    else if((std_id == SBV_CAN_STD_ID_TUNNING)
            || (std_id == SBV_CAN_STD_ID_COMMAND)
            || (std_id == SBV_CAN_STD_ID_LOGGING))
    {
        return;
    }
}