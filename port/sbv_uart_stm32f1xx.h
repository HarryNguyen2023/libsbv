#ifndef __SBV_UART_STM32F1XX_H__
#define __SBV_UART_STM32F1XX_H__

#include "sbv.h"
#include "sbv_uart.h"

#ifdef STM32F1xx

#include "stm32f1xx_hal_uart.h"

#define SBV_UART_TX_BUFFER_SIZE     512
#define SBV_UART_RX_BUFFER_SIZE     512

typedef UART_HandleTypeDef  sbv_uart_handle_t;
typedef DMA_HandleTypeDef   sbv_uart_dma_handle_t;

typedef struct sbv_uart_instance_t
{
    uint8_t                 rx_isr_registered;
    uint8_t                 *uart_tx_buffer;  
    sbv_cqbuff*             uart_rx_buffer;
    sbv_uart_handle_t*      uart_handle;
    sbv_uart_dma_handle_t*  uart_rx_dma_handle;
    sbv_uart_baudrate_t     uart_baudrate;
    sbv_rtos_task_handle_t  uart_rx_notify_task;
    sbv_rtos_tick_type_t    uart_tx_timeout;
    uint16_t                uart_rx_isr_size;
    int                     (*uart_rx_cb)(uint8_t *, const uint16_t);
} sbv_uart_instance_t;

void
sbv_uart_stm32f1xx_init (void *unused, sbv_uart_handle_t* uart_handle,
                         sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate);
int
sbv_uart_stm32f1xx_send_data (sbv_uart_handle_t* uart_handle, uint8_t* uart_tx_data, uint16_t uart_tx_size);
uint8_t *
sbv_uart_stm32f1xx_rcv_data (uint16_t *size, uint16_t timeout_ms);
int
sbv_uart_stm32f1xx_rx_read_data (uint8_t *data, int size);
int
sbv_uart_stm32f1xx_register_rx_cb (int (*uart_rx_cb)(uint8_t *, const uint16_t));
#endif /* STM32F1xx */
#endif /* __SBV_UART_STM32F1XX_H__ */