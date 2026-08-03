#ifndef __SBV_UART_ESP32S3_H__
#define __SBV_UART_ESP32S3_H__

#include "sbv.h"
#include "sbv_uart.h"

#ifdef ESP32xx_IDF
#include "hal/uart_ll.h"
#include "soc/interrupts.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"

#define SBV_UART_RX_BUFFER_SIZE     512
#define SBV_UART_TX_BUFFER_SIZE     512
#define SBV_UART_MAX_CHANNEL        3
#define SBV_UART_MAX_WRITE_TRY      5

typedef uart_config_t       sbv_uart_cfg_t;
typedef uart_port_t         sbv_uart_handle_t;
typedef intr_handle_t       sbv_uart_dma_handle_t;
typedef uart_intr_config_t  sbv_uart_intr_config_t;

typedef struct sbv_uart_instance_t
{
    sbv_rtos_mutex_t        mu;  
    sbv_cqbuff*             uart_rx_buffer;
    sbv_uart_handle_t*      uart_handle;
    sbv_uart_dma_handle_t*  uart_rx_dma_handle;
    sbv_uart_baudrate_t     uart_baudrate;
    sbv_rtos_task_handle_t  uart_rx_notify_task;
    int                     (*uart_rx_cb)(uint8_t *, const uint16_t);
} sbv_uart_instance_t;

struct sbv_uart_instances_list_t {
    sbv_uart_instance_t* list[SBV_UART_MAX_CHANNEL];
};

#define sbv_uart_esp32s3_driver_install(P, RSIZE, TSIZE, QSIZE, QUEUE, ISR)  \
        uart_driver_install(P, RSIZE, TSIZE, QSIZE, QUEUE, ISR)

#define sbv_uart_esp32s3_param_config(P, C)  \
        uart_param_config(P, C)

#define sbv_uart_esp32s3_set_pin(P, TPIN, RPIN, RTSPIN, CTSPIN) \
        uart_set_pin(P, TPIN, RPIN, RTSPIN, CTSPIN)

#define sbv_uart_esp32s3_intr_alloc(S, ARG, ISR, ALLOC, H)  \
        esp_intr_alloc(S, ARG, ISR, ALLOC, H)

#define sbv_uart_esp32s3_enable_rx_intr(P)  \
        uart_enable_rx_intr(P)

#define sbv_uart_esp32s3_clear_intr_status(P, S)  \
        uart_clear_intr_status(P, S)

#define sbv_uart_esp32s3_intr_config(P, C)  \
        uart_intr_config(P, C)


int
sbv_uart_esp32s3_init(sbv_uart_instance_t *uart_instance, sbv_uart_handle_t* uart_handle,
                      sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate,
                      sbv_gpio_num_t uart_pin[2]);

int
sbv_uart_esp32s3_send_data (sbv_uart_instance_t* uart_instance,
                            uint8_t* uart_tx_data,
                            uint16_t uart_tx_size, uint16_t timeout_ms);
int
sbv_uart_esp32s3_rcv_data (sbv_uart_instance_t* uart_instance,
                           uint8_t recv_buff[], uint16_t size,
                           uint16_t timeout_ms);
#endif /* ESP32xx_IDF */
#endif /* __SBV_UART_ESP32S3_H__ */