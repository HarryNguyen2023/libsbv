#include <stdlib.h>
#include <string.h>
#include "sbv_rtos.h"
#include "sbv_uart.h"
#include "sbv_uart_stm32f1xx.h"

#ifdef STM32F1xx

struct sbv_uart_instances_list_t sbv_uart_instances_list = {0};

#define SBV_UART_MUTEX_LOCK(Instance) \
        sbv_rtos_mutex_lock(Instance->mutex)

#define SBV_UART_MUTEX_UNLOCK(Instance) \
        sbv_rtos_mutex_unlock(Instance->mutex)

#define sbv_uart_stm32f1xx_rx_hw_callback \
        HAL_UARTEx_RxEventCallback

/* Return the UART instance that owns the given HAL UART handle. */
static sbv_uart_instance_t*
sbv_uart_stm32f1xx_get_instance_by_handle (sbv_uart_handle_t* uart_handle)
{
    if (! uart_handle)
        return NULL;

    for (uint8_t i = 0; i < SBV_UART_MAX_CHANNEL; ++i)
    {
        if (sbv_uart_instances_list.list[i]
            && sbv_uart_instances_list.list[i]->uart_handle == uart_handle)
        {
           return sbv_uart_instances_list.list[i];
        }
    }

    return NULL;
}

/* Register a new UART instance in the internal instance list. */
static int
sbv_uart_stm32f1xx_add_instance_to_list (sbv_uart_instance_t *uart_instance)
{
    if (! uart_instance)
        return SBV_ERROR;

    for (uint8_t i = 0; i < SBV_UART_MAX_CHANNEL; ++i)
    {
        if (sbv_uart_instances_list.list[i] == NULL)
        {
           sbv_uart_instances_list.list[i] =  uart_instance;
           return SBV_OK;
        }
    }

    return SBV_ERROR;
}

static void
sbv_uart_stm32f1xx_rx_idle_deteciton_start (sbv_uart_handle_t* uart_handle,
                                            sbv_uart_dma_handle_t* uart_dma_handle,
                                            uint8_t* rcv_buffer, uint16_t rcv_buffer_size)
{
    if(!rcv_buffer || !uart_handle || !uart_dma_handle)
        return;

    /* Enable the idle line detection */
    HAL_UARTEx_ReceiveToIdle_DMA(uart_handle, rcv_buffer, rcv_buffer_size);
    /* Since the DMA module in STM32 include 2 interrupt service: half recption 
     * and full reception, since we only care about full reception we will disable
     * the half reception interrupt to free the CPU handle
     */
    __HAL_DMA_DISABLE_IT(uart_dma_handle, DMA_IT_HT);
}

int
sbv_uart_stm32f1xx_init (sbv_uart_instance_t *uart_instance, sbv_uart_handle_t* uart_handle,
                         sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate)
{
    if(! uart_instance || ! uart_handle || ! uart_dma_handle)
        return SBV_ERROR;

    if (sbv_uart_stm32f1xx_add_instance_to_list (uart_instance) != SBV_OK)
    {
        /* LOG */
        return SBV_ERROR;
    }

    /* Initiate the rx instance */
    uart_instance->uart_rx_cb             = NULL;
    uart_instance->uart_rx_buffer         = sbv_cqbuff_create (SBV_UART_RX_BUFFER_SIZE, sizeof (uint8_t));
    if (! uart_instance->uart_rx_buffer)
    {
        /* LOG */
        return SBV_ERROR;
    }
    uart_instance->uart_rx_buffer->head   = 0;
    uart_instance->uart_rx_buffer->rear   = 0;

    uart_instance->uart_handle            = uart_handle;
    uart_instance->uart_rx_dma_handle     = uart_dma_handle;

    uart_instance->uart_baudrate          = baudrate;
    uart_instance->uart_rx_notify_task    = NULL;

    /* Create the mutex for the UART channel */
    sbv_rtos_mutex_create(uart_instance->mutex);

    /* Start to register for UART DMA Idle line Interrupt callback */
    sbv_uart_stm32f1xx_rx_idle_deteciton_start (uart_handle, uart_dma_handle,
                                                uart_instance->uart_rx_buffer->buff,
                                                SBV_UART_RX_BUFFER_SIZE);

    return SBV_OK;
}

static int
sbv_uart_stm32f1xx_tx_send_pkt(sbv_uart_handle_t* uart_handle, uint8_t* uart_tx_buffer,
                               uint16_t uart_tx_size, uint16_t timeout_ms)
{
    int ret = SBV_OK;

    if(! uart_handle || ! uart_tx_buffer)
        return SBV_ERROR;

    ret = HAL_UART_Transmit(uart_handle, uart_tx_buffer, uart_tx_size, timeout_ms);

    return ret;
}

int
sbv_uart_stm32f1xx_send_data(sbv_uart_instance_t* uart_instance, uint8_t* uart_tx_data,
                            uint16_t uart_tx_size, uint16_t timeout_ms)
{
    int ret = SBV_OK;

    if(! uart_instance || ! uart_tx_data)
        return SBV_ERROR;

    SBV_UART_MUTEX_LOCK (uart_instance);

    ret = sbv_uart_stm32f1xx_tx_send_pkt(uart_instance->uart_handle,
                                         uart_tx_data, uart_tx_size, timeout_ms);
    if (ret != SBV_OK)
    {
        // LOG
        SBV_UART_MUTEX_UNLOCK (uart_instance);
        return ret;
    }

    SBV_UART_MUTEX_UNLOCK (uart_instance);

    return ret;
}

void
sbv_uart_stm32f1xx_rx_hw_callback(sbv_uart_handle_t* uart_handle, uint16_t uart_rx_size)
{
    sbv_uart_instance_t* uart_instance = NULL;
    sbv_rtos_base_type_t xHigherPriorityTaskWoken = SBV_RTOS_FALSE;
    uint16_t rx_buffer_size_left;
    uint8_t* rx_buffer_cur_pos;

    if(! uart_handle)
        return;

    uart_instance = sbv_uart_stm32f1xx_get_instance_by_handle (uart_handle);
    if (! uart_instance)
    {
        /* LOG */
        return;
    }

    uart_instance->uart_rx_buffer->head = (uart_instance->uart_rx_buffer->head + \
                                            uart_rx_size) % uart_instance->uart_rx_buffer->capacity;

    // Continue to register for UART DMA Idle line interrupt callback
    rx_buffer_cur_pos = uart_instance->uart_rx_buffer->buff + \
                            uart_instance->uart_rx_buffer->head;
    rx_buffer_size_left = sbv_cqbuff_avail_size (uart_instance->uart_rx_buffer);

    sbv_uart_stm32f1xx_rx_idle_deteciton_start (uart_instance->uart_handle,
                                                uart_instance->uart_rx_dma_handle, 
                                                rx_buffer_cur_pos, rx_buffer_size_left);
    if(uart_instance->uart_rx_notify_task != NULL)
    {
        sbv_rtos_notify_give_fromISR (uart_instance->uart_rx_notify_task,
                                      &xHigherPriorityTaskWoken);
        sbv_rtos_port_yield_fromISR (xHigherPriorityTaskWoken);
    }
}

int
sbv_uart_stm32f1xx_rcv_data (sbv_uart_instance_t* uart_instance,
                             uint8_t recv_buff[], uint16_t size,
                             uint16_t timeout_ms)
{
    uint8_t* rx_buffer_ret_pos;
    uint16_t rx_buffer_size;
    uint32_t notify;
    sbv_rtos_tick_type_t tick_to_wait;

    if (! uart_instance)
        return SBV_ERROR;

    tick_to_wait = sbv_rtos_ms_to_tick(timeout_ms);

    SBV_UART_MUTEX_LOCK (uart_instance);

    uart_instance->uart_rx_notify_task = sbv_rtos_get_current_task_handle();

    /* Blocking call until timeout */
    notify = sbv_rtos_notify_take(SBV_RTOS_TRUE, tick_to_wait);
    if (notify == 0)
    {
        /* No notification is received after the timeout event */
        uart_instance->uart_rx_notify_task = NULL;
        SBV_UART_MUTEX_UNLOCK (uart_instance);
        return 0;
    }

    uart_instance->uart_rx_notify_task = NULL;

    rx_buffer_ret_pos = uart_instance->uart_rx_buffer->buff + \
                            uart_instance->uart_rx_buffer->rear;

    rx_buffer_size = sbv_cqbuff_get_size (uart_instance->uart_rx_buffer);

    if (uart_instance->uart_rx_cb)
    {
        (*uart_instance->uart_rx_cb) (rx_buffer_ret_pos, rx_buffer_size);
        /* Update current rear pointer position */
        uart_instance->uart_rx_buffer->rear = (uart_instance->uart_rx_buffer->rear + rx_buffer_size) \
                                                % uart_instance->uart_rx_buffer->capacity;
    }
    else
    {
        if (recv_buff && size > 0)
        {
            rx_buffer_size = (rx_buffer_size < size) ? rx_buffer_size : size;
            sbv_cqbuff_read (uart_instance->uart_rx_buffer, recv_buff, rx_buffer_size);
        }
    }

    SBV_UART_MUTEX_UNLOCK (uart_instance);

    return rx_buffer_size;
}

int
sbv_uart_stm32f1xx_register_rx_cb (sbv_uart_instance_t* uart_instance,
                                   int (*uart_rx_cb)(uint8_t *, const uint16_t))
{
    if (! uart_rx_cb || ! uart_instance)
        return -1;

    SBV_UART_MUTEX_LOCK (uart_instance);

    uart_instance->uart_rx_cb = uart_rx_cb;

    SBV_UART_MUTEX_UNLOCK (uart_instance);
    return 0;
}
#endif /* STM32F1xx */