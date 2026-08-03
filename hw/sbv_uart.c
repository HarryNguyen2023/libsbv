#include "stdio.h"
#include "string.h"
#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_uart.h"
#include "sbv_gpio.h"

struct sbv_uart_hw_cb_t sbv_uart_hw_cb = {
#ifdef STM32F1xx
    .sbv_uart_init              = sbv_uart_stm32f1xx_init,
    .sbv_uart_tx_send_data      = sbv_uart_stm32f1xx_send_data,
    .sbv_uart_rx_rcv_data       = sbv_uart_stm32f1xx_rcv_data,
    .sbv_uart_register_rx_cb    = sbv_uart_stm32f1xx_register_rx_cb,
#elif defined ESP32xx_IDF
    .sbv_uart_init              = sbv_uart_esp32s3_init,
    .sbv_uart_tx_send_data      = sbv_uart_esp32s3_send_data,
    .sbv_uart_rx_rcv_data       = sbv_uart_esp32s3_rcv_data,
#endif /* STM32F1xx */
};

int
sbv_uart_init (sbv_uart_instance_t *uart_instance, sbv_uart_handle_t* uart_handle,
               sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate,
               void *uart_pin)
{
    if (sbv_uart_hw_cb.sbv_uart_init)
        return (sbv_uart_hw_cb.sbv_uart_init) (uart_instance, uart_handle, uart_dma_handle, baudrate, uart_pin);

    return SBV_ERROR;
}

int
sbv_uart_tx_send_data(sbv_uart_instance_t *uart_instance, uint8_t* uart_tx_data,
                      uint16_t uart_tx_size, uint16_t timeout_ms)
{
    if (sbv_uart_hw_cb.sbv_uart_tx_send_data)
        return (sbv_uart_hw_cb.sbv_uart_tx_send_data) (uart_instance, uart_tx_data,
                                                       uart_tx_size, timeout_ms);

    return SBV_ERROR;
}

int
sbv_uart_rx_rcv_data (sbv_uart_instance_t* uart_instance,
                      uint8_t recv_buff[], uint16_t size,
                      uint16_t timeout_ms)
{
    if (sbv_uart_hw_cb.sbv_uart_rx_rcv_data)
        return (sbv_uart_hw_cb.sbv_uart_rx_rcv_data) (uart_instance, recv_buff,
                                                      size, timeout_ms);
    return SBV_ERROR;
}

int
sbv_uart_register_rx_cb (sbv_uart_instance_t *uart_instance, int (*uart_rx_cb)(uint8_t *, const uint16_t))
{
    if (sbv_uart_hw_cb.sbv_uart_register_rx_cb)
        return (sbv_uart_hw_cb.sbv_uart_register_rx_cb) (uart_instance, uart_rx_cb);

    return SBV_ERROR;
}

int
sbv_uart_send_ota_data (sbv_uart_instance_t *sbv_uart_instance, uint8_t type, uint8_t* data, uint16_t length, uint16_t timeout_ms)
{
    if (sbv_uart_hw_cb.sbv_uart_tx_send_data)
        return (sbv_uart_hw_cb.sbv_uart_tx_send_data) (sbv_uart_instance, data, length, timeout_ms);

    return SBV_ERROR;
}