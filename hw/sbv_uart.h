#ifndef SBV_UART_H
#define SBV_UART_H

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_gpio.h"
#include "sbv_cqbuff.h"

typedef enum sbv_uart_baudrate_t
{
    SBV_UART_BAUDRATE_9600      = 9600,
    SBV_UART_BAUDRATE_19200     = 19200, 
    SBV_UART_BAUDRATE_38400     = 38400,
    SBV_UART_BAUDRATE_57600     = 57600,
    SBV_UART_BAUDRATE_115200    = 115200,
} sbv_uart_baudrate_t;

#ifdef STM32F1xx
#include "sbv_uart_stm32f1xx.h"
#elif defined ESP32xx_IDF
#include "sbv_uart_esp32s3.h"
#endif

struct sbv_uart_hw_cb_t
{
    sbv_uart_instance_t* (*sbv_uart_init) (void *, sbv_uart_handle_t *, sbv_uart_dma_handle_t *, sbv_uart_baudrate_t);
    int (*sbv_uart_tx_send_data) (sbv_uart_instance_t *, uint8_t *, uint16_t, uint16_t);
    uint8_t* (*sbv_uart_rx_rcv_data) (sbv_uart_instance_t *, uint16_t *, uint16_t);
    int (*sbv_uart_register_rx_cb) (sbv_uart_instance_t *, int (*uart_rx_cb)(uint8_t *, const uint16_t));
};

sbv_uart_instance_t *
sbv_uart_init (void *uart_pin, sbv_uart_handle_t* uart_handle,
               sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate);
int
sbv_uart_tx_send_data(sbv_uart_instance_t *, uint8_t* uart_tx_data, uint16_t uart_tx_size, uint16_t timeout_ms);
uint8_t *
sbv_uart_rx_rcv_data (sbv_uart_instance_t *, uint16_t *size, uint16_t timeout_ms);
int
sbv_uart_register_rx_cb (sbv_uart_instance_t *, int (*uart_rx_cb)(uint8_t *, const uint16_t));
int
sbv_uart_send_ota_data (sbv_uart_instance_t *, uint8_t type, uint8_t* data, uint16_t length, uint16_t timeout_ms);

#endif /*SBV_UART_H*/