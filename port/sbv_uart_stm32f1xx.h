#ifndef __SBV_UART_STM32F1XX_H__
#define __SBV_UART_STM32F1XX_H__

#include "sbv.h"
#include "sbv_uart.h"

#ifdef STM32F1xx

#include "stm32f1xx_hal_uart.h"
#define SBV_UART_RX_BUFFER_SIZE     512
#define SBV_UART_MAX_CHANNEL        3

typedef UART_HandleTypeDef  sbv_uart_handle_t;
typedef DMA_HandleTypeDef   sbv_uart_dma_handle_t;
typedef struct sbv_uart_instance_t sbv_uart_instance_t;

struct sbv_uart_instance_t
{
    sbv_rtos_mutex_t        mutex;  
    sbv_cqbuff*             uart_rx_buffer;
    sbv_uart_handle_t*      uart_handle;
    sbv_uart_dma_handle_t*  uart_rx_dma_handle;
    sbv_uart_baudrate_t     uart_baudrate;
    sbv_rtos_task_handle_t  uart_rx_notify_task;
    int                     (*uart_rx_cb)(uint8_t *, const uint16_t);
};

struct sbv_uart_instances_list_t {
    sbv_uart_instance_t* list[SBV_UART_MAX_CHANNEL];
};

int
sbv_uart_stm32f1xx_init (sbv_uart_instance_t *uart_instance, sbv_uart_handle_t* uart_handle,
                         sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate);
int
sbv_uart_stm32f1xx_send_data (sbv_uart_instance_t* sbv_uart_instance, uint8_t* uart_tx_data,
                              uint16_t uart_tx_size, uint16_t timeout_ms);
int
sbv_uart_stm32f1xx_rcv_data (sbv_uart_instance_t* uart_instance,
                             uint8_t recv_buff[], uint16_t size,
                             uint16_t timeout_ms);
int
sbv_uart_stm32f1xx_register_rx_cb (sbv_uart_instance_t* sbv_uart_instance, int (*uart_rx_cb)(uint8_t *, const uint16_t));
#endif /* STM32F1xx */
#endif /* __SBV_UART_STM32F1XX_H__ */