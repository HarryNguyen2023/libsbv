#include "stdio.h"
#include "string.h"
#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_uart.h"
#include "sbv_gpio.h"

extern sbv_uart_instance_t* sbv_uart_instance;

sbv_rtos_mutex_t SBV_UART_TX_BUFFER_MUTEX;
sbv_rtos_mutex_t SBV_UART_RX_BUFFER_MUTEX;

struct sbv_uart_hw_cb_t sbv_uart_hw_cb = {
#ifdef STM32F1xx
    .sbv_uart_init              = sbv_uart_stm32f1xx_init,
    .sbv_uart_tx_send_data      = sbv_uart_stm32f1xx_send_data,
    .sbv_uart_rx_rcv_data       = sbv_uart_stm32f1xx_rcv_data,
    .sbv_uart_rx_read_data      = sbv_uart_stm32f1xx_rx_read_data,
    .sbv_uart_register_rx_cb    = sbv_uart_stm32f1xx_register_rx_cb,
#elif defined ESP32xx_IDF
    .sbv_uart_init              = sbv_uart_esp32s3_init,
    .sbv_uart_tx_send_data      = sbv_uart_esp32s3_send_data,
    .sbv_uart_rx_rcv_data       = sbv_uart_esp32s3_rcv_data,
#endif /* STM32F1xx */
};

void
sbv_uart_init (void *uart_pin, sbv_uart_handle_t* uart_handle,
               sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate)
{
    if (sbv_uart_hw_cb.sbv_uart_init)
        (sbv_uart_hw_cb.sbv_uart_init) (uart_pin, uart_handle, uart_dma_handle, baudrate);
}

int
sbv_uart_tx_send_data(uint8_t* uart_tx_data, uint16_t uart_tx_size)
{
    if (! sbv_uart_instance)
        return 0;

    if (sbv_uart_hw_cb.sbv_uart_tx_send_data)
        return (sbv_uart_hw_cb.sbv_uart_tx_send_data) (sbv_uart_instance->uart_handle, uart_tx_data, uart_tx_size);

    return 0;
}

uint8_t *
sbv_uart_rx_rcv_data (uint16_t *size)
{
    if (sbv_uart_hw_cb.sbv_uart_rx_rcv_data)
        return (sbv_uart_hw_cb.sbv_uart_rx_rcv_data) (size);

    *size = 0;
    return NULL;
}

int
sbv_uart_rx_read_data (uint8_t *data, int size)
{
    if (sbv_uart_hw_cb.sbv_uart_rx_read_data)
        return (sbv_uart_hw_cb.sbv_uart_rx_read_data) (data, size);

    return 0;
}

void
sbv_uart_send_debug (uint8_t *data, uint16_t data_length)
{
    sbv_uart_tx_send_data (data, data_length);
}

int
sbv_uart_register_rx_cb (int (*uart_rx_cb)(uint8_t *, const uint16_t))
{
    if (sbv_uart_hw_cb.sbv_uart_register_rx_cb)
        return (sbv_uart_hw_cb.sbv_uart_register_rx_cb) (uart_rx_cb);

    return 0;
}

int
sbv_uart_send_ota_data (uint8_t type, uint8_t* data, uint16_t length)
{
    if (! sbv_uart_instance)
        return 0;

    if (sbv_uart_hw_cb.sbv_uart_tx_send_data)
        return (sbv_uart_hw_cb.sbv_uart_tx_send_data) (sbv_uart_instance->uart_handle, data, length);

    return 0;
}