#ifndef __SBV_UART_ESP32S3_H__
#define __SBV_UART_ESP32S3_H__

#include "sbv.h"
#include "sbv_uart.h"

#ifdef ESP32xx_IDF
#include "hal/uart_ll.h"
#include "soc/interrupts.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"

typedef uart_config_t       sbv_uart_cfg_t;
typedef uart_port_t         sbv_uart_handle_t;
typedef intr_handle_t       sbv_uart_dma_handle_t;
typedef uart_intr_config_t  sbv_uart_intr_config_t;

typedef struct sbv_uart_instance_t
{
    uint8_t                 *uart_rx_buffer;
    uint8_t                 *uart_tx_buffer;        
    sbv_uart_handle_t*      uart_handle;
    sbv_uart_dma_handle_t*  uart_rx_dma_handle;
    sbv_uart_baudrate_t     uart_baudrate;
    sbv_rtos_task_handle_t  uart_rx_notify_task;
    sbv_rtos_tick_type_t    uart_rx_timeout;
    sbv_rtos_tick_type_t    uart_tx_timeout;
    uint16_t                uart_rx_buffer_pos;
    uint16_t                uart_rx_isr_size;
} sbv_uart_instance_t;

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


void
sbv_uart_esp32s3_init(sbv_gpio_num_t uart_pin[2], sbv_uart_handle_t* uart_port,
                      sbv_uart_dma_handle_t* uart_intr_handle, sbv_uart_baudrate_t baudrate);

int
sbv_uart_esp32s3_send_data(sbv_uart_handle_t* uart_handle, uint8_t* uart_tx_data, uint16_t uart_tx_size);
uint8_t *
sbv_uart_esp32s3_rcv_data (uint16_t *size);
#endif /* ESP32xx_IDF */
#endif /* __SBV_UART_ESP32S3_H__ */