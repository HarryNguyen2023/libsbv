#ifndef SBV_CAN_H
#define SBV_CAN_H

#include "sbv.h"
#include "sbv_rtos.h"

typedef enum sbv_can_msg_type_t
{
    SBV_CAN_MSG_TUNNING = 0,
    SBV_CAN_MSG_COMMAND,
    SBV_CAN_MSG_LOGGING,
    SBV_CAN_MSG_OTA
} sbv_can_msg_type_t;

#ifdef STM32F1xx
#include "sbv_can_stm32f1xx.h"
#elif defined ESP32xx_IDF
#include "sbv_can_esp32s3.h"
#endif /*STM32F1xx*/

#define SBV_CAN_RX_BUFFER_MUTEX_LOCK \
        sbv_rtos_mutex_lock(SBV_CAN_RX_BUFFER_MUTEX)

#define SBV_CAN_RX_BUFFER_MUTEX_UNLOCK \
        sbv_rtos_mutex_unlock(SBV_CAN_RX_BUFFER_MUTEX)

#define SBV_CAN_TX_BUFFER_MUTEX_LOCK \
        sbv_rtos_mutex_lock(SBV_CAN_TX_BUFFER_MUTEX)

#define SBV_CAN_TX_BUFFER_MUTEX_UNLOCK \
        sbv_rtos_mutex_unlock(SBV_CAN_TX_BUFFER_MUTEX)

#define SBV_CAN_RX_TIMEOUT  sbv_rtos_ms_to_tick(1000)
#define SBV_CAN_TX_TIMEOUT  sbv_rtos_ms_to_tick(10)

#define SBV_CAN_STD_ID_BASE             (0x5)
#define SBV_CAN_STD_ID_BASE_OFFSET      (8)

#define SBV_CAN_STD_ID_START            (0)
#define SBV_CAN_STD_ID_TUNNING          (SBV_CAN_STD_ID_START + 1)
#define SBV_CAN_STD_ID_COMMAND          (SBV_CAN_STD_ID_START + 2)
#define SBV_CAN_STD_ID_LOGGING          (SBV_CAN_STD_ID_START + 3)
#define SBV_CAN_STD_ID_OTA              (SBV_CAN_STD_ID_START + 4)

#define SBV_CAN_STD_ID_OFFSET           (5)
#define SBV_CAN_STD_ID_NODE_ID_OFFSET   (0)
#define SBV_CAN_STD_ID_FILTER_ID        (SBV_CAN_STD_ID_BASE << SBV_CAN_STD_ID_BASE_OFFSET)
#define SBV_CAN_STD_ID_MASK             (0x7 << SBV_CAN_STD_ID_BASE_OFFSET)

typedef struct sbv_can_hw_cb_t
{
    void (*sbv_can_init) (void *);
    int (*sbv_can_send_data) (sbv_can_msg_type_t, uint8_t *, uint8_t);
    uint8_t* (*sbv_can_rcv_data) (uint8_t *, uint16_t *);
    uint32_t (*sbv_can_std_id_get) (uint32_t);
} sbv_can_hw_cb_t;

void
sbv_can_init(void *can_handle);
int
sbv_can_send_data(sbv_can_msg_type_t msg_type, uint8_t *data, uint8_t length);
uint8_t*
sbv_can_rcv_data(uint8_t *length,  uint16_t *std_id);
void
sbv_can_pkt_process(uint8_t *data, uint8_t length, uint16_t std_id);

#endif /*SBV_CAN_H*/