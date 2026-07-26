#include <stdlib.h>
#include <string.h>
#include "sbv_rtos.h"
#include "sbv_uart.h"
#include "sbv_uart_stm32f1xx.h"

#ifdef STM32F1xx

#define SBV_UART_MAX_CHANNEL    (3)

// A linked list that maintain HW uart channels
struct sbv_uart_hw_instances_t {
    sbv_uart_instance_t *head;
    uint8_t             count;
};

struct sbv_uart_hw_instances_t sbv_uart_hw_instances = {0};

#define sbv_uart_stm32f1xx_rx_hw_callback \
        HAL_UARTEx_RxEventCallback

extern sbv_rtos_mutex_t SBV_UART_TX_BUFFER_MUTEX;
extern sbv_rtos_mutex_t SBV_UART_RX_BUFFER_MUTEX;

static uint8_t
sbv_uart_stm32f1xx_instance_exist (sbv_uart_handle_t* uart_handle,
                                   sbv_uart_dma_handle_t* uart_dma_handle)
{
    sbv_uart_instance_t *head;

    if (! uart_handle || ! uart_dma_handle)
        return 1;

    for (head = sbv_uart_hw_instances.head; head != NULL; head = head->next)
    {
        if (head->uart_handle->Instance == uart_handle->Instance
            || head->uart_rx_dma_handle->Instance == uart_dma_handle->Instance)
            return 1;
    }

    return 0;
}

static uint8_t
sbv_uart_stm32f1xx_instance_add (sbv_uart_instance_t* sbv_uart_instance)
{
    sbv_uart_instance_t *head;

    if (! sbv_uart_instance)
        return -1;

    if (sbv_uart_hw_instances.count >= SBV_UART_MAX_CHANNEL)
    {
        /* LOG */
        return -1;
    }

    head = sbv_uart_hw_instances.head;
    if (head == NULL)
    {
        sbv_uart_hw_instances.head = sbv_uart_instance;
    }
    else
    {
        while (head->next)
            head = head->next;

        head->next = sbv_uart_instance;
    }
    (sbv_uart_hw_instances.count)++;
    /* LOG */

    return 0;
}

static uint8_t
sbv_uart_stm32f1xx_instance_remove (sbv_uart_instance_t* sbv_uart_instance)
{
    sbv_uart_instance_t *head;
    uint8_t found = SBV_FALSE;

    if (! sbv_uart_instance)
        return -1;

    if (sbv_uart_hw_instances.count == 0)
    {
        /* LOG */
        return -1;
    }

    head = sbv_uart_hw_instances.head;
    if (head == NULL)
    {
        /* LOG */
        return -1;
    }
    
    if (head == sbv_uart_instance)
    {
        sbv_uart_hw_instances.head = head->next;
        found = SBV_TRUE;
    }
    else
    {
        while (head->next != sbv_uart_instance)
            head = head->next;
        
        if (head->next == sbv_uart_instance)
        {
            head->next = head->next->next;
            found = SBV_TRUE;
        }
    }

    if (found)
    {
        (sbv_uart_hw_instances.count)--;
        /* LOG */
    }
    else
    {
        /* LOG */
    }

    return 0;
}

static sbv_uart_instance_t *
sbv_uart_stm32f1xx_instance_lookup (sbv_uart_handle_t* uart_handle)
{
    sbv_uart_instance_t *head;

    if (! uart_handle)
        return NULL;

    if (sbv_uart_hw_instances.count == 0)
    {
        /* LOG */
        return NULL;
    }

    for (head = sbv_uart_hw_instances.head; head != NULL; head = head->next)
    {
        if (head->uart_handle->Instance == uart_handle->Instance)
            return head;
    }

    return NULL;
}

static void
sbv_uart_stm32f1xx_rx_idle_deteciton_start (sbv_uart_handle_t* uart_handle, sbv_uart_dma_handle_t* uart_dma_handle,
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

sbv_uart_instance_t *
sbv_uart_stm32f1xx_init (void *unused, sbv_uart_handle_t* uart_handle,
                         sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate)
{
    sbv_uart_instance_t* sbv_uart_instance;

    if(! uart_handle || ! uart_dma_handle)
        return NULL;

    if (sbv_uart_stm32f1xx_instance_exist (uart_handle, uart_dma_handle))
    {
        /* LOG */
        return NULL;
    }

    sbv_uart_instance = (sbv_uart_instance_t *)sbv_rtos_malloc(sizeof (sbv_uart_instance_t));
    if (! sbv_uart_instance)
    {
        /* LOG */
        return NULL;
    }
    /* Initiate the rx instance */
    sbv_uart_instance->rx_isr_registered      = SBV_FALSE;
    sbv_uart_instance->uart_rx_cb             = NULL;
    sbv_uart_instance->uart_rx_buffer         = sbv_cqbuff_create (SBV_UART_RX_BUFFER_SIZE, sizeof (uint8_t));
    if (! sbv_uart_instance->uart_rx_buffer)
    {
        /* LOG */
        goto ERR_EXIT;
    }
    sbv_uart_instance->uart_tx_buffer         = (uint8_t *)sbv_rtos_malloc(sizeof (uint8_t) * SBV_UART_TX_BUFFER_SIZE);
    if (! sbv_uart_instance->uart_tx_buffer)
    {
        /* LOG */
        goto ERR_EXIT;
    }
    sbv_uart_instance->uart_handle            = uart_handle;
    sbv_uart_instance->uart_rx_dma_handle     = uart_dma_handle;

    sbv_uart_instance->uart_baudrate          = baudrate;
    sbv_uart_instance->uart_rx_notify_task    = NULL;
    sbv_uart_instance->uart_rx_isr_size       = 0;
    sbv_uart_instance->next                   = NULL;

    /* Create the mutex for the UART channel */
    sbv_rtos_mutex_create(sbv_uart_instance->mutex);

    /* Initiate the tx and rx buffers */
    memset(sbv_uart_instance->uart_tx_buffer, 0, SBV_UART_TX_BUFFER_SIZE);

    sbv_uart_stm32f1xx_instance_add (sbv_uart_instance);

    return sbv_uart_instance;

ERR_EXIT:
    if (sbv_uart_instance->uart_rx_buffer)  sbv_rtos_free (sbv_uart_instance->uart_rx_buffer);
    sbv_uart_instance->uart_rx_buffer = NULL;
    if (sbv_uart_instance)                  sbv_rtos_free (sbv_uart_instance);
    sbv_uart_instance = NULL;
    return NULL;
}

static int
sbv_uart_stm32f1xx_tx_send_pkt(sbv_uart_handle_t* uart_handle, uint8_t* uart_tx_buffer, uint16_t uart_tx_size, uint16_t timeout_ms)
{
    int ret = SBV_OK;

    if(! uart_handle || ! uart_tx_buffer)
        return SBV_ERROR;

    ret = HAL_UART_Transmit(uart_handle, uart_tx_buffer, uart_tx_size, timeout_ms);

    return ret;
}

int
sbv_uart_stm32f1xx_tx_packet_format(uint8_t* uart_tx_data, uint16_t uart_tx_size, uint8_t* uart_tx_buffer)
{
    int sent_bytes = 0;

    if(! uart_tx_data || ! uart_tx_buffer)
        return 0;

    memset(uart_tx_buffer, 0, SBV_UART_TX_BUFFER_SIZE);

    if(uart_tx_size <= SBV_UART_TX_BUFFER_SIZE)
    {
        memcpy(uart_tx_buffer, uart_tx_data, uart_tx_size);
        sent_bytes = uart_tx_size;
    }
    else
    {
        memcpy(uart_tx_buffer, uart_tx_data, SBV_UART_TX_BUFFER_SIZE);
        sent_bytes = SBV_UART_TX_BUFFER_SIZE;
    }

    return sent_bytes;
}

int
sbv_uart_stm32f1xx_send_data(sbv_uart_instance_t* sbv_uart_instance, uint8_t* uart_tx_data, uint16_t uart_tx_size, uint16_t timeout_ms)
{
    int ret = SBV_OK, total_tx_bytes = 0, cur_tx_bytes = 0;

    if(! sbv_uart_instance || ! uart_tx_data)
        return 0;

    sbv_rtos_mutex_lock (sbv_uart_instance->mutex);

    while (total_tx_bytes < uart_tx_size)
    {
        cur_tx_bytes = sbv_uart_stm32f1xx_tx_packet_format(uart_tx_data + total_tx_bytes,
                                                          (uart_tx_size - total_tx_bytes),
                                                          sbv_uart_instance->uart_tx_buffer);
        ret = sbv_uart_stm32f1xx_tx_send_pkt(sbv_uart_instance->uart_handle, sbv_uart_instance->uart_tx_buffer, cur_tx_bytes, timeout_ms);
        if (ret != SBV_OK)
            continue;
        total_tx_bytes += cur_tx_bytes;
    }

    sbv_rtos_mutex_unlock (sbv_uart_instance->mutex);

    return total_tx_bytes;
}

void
sbv_uart_stm32f1xx_rx_hw_callback(sbv_uart_handle_t* uart_handle, uint16_t uart_rx_size)
{
    sbv_uart_instance_t* sbv_uart_instance = NULL;
    sbv_rtos_base_type_t xHigherPriorityTaskWoken = SBV_RTOS_FALSE;

    if(! uart_handle)
        return;

    sbv_uart_instance = sbv_uart_stm32f1xx_instance_lookup (uart_handle);
    if (! sbv_uart_instance)
    {
        /* LOG */
        return;
    }

    sbv_uart_instance->uart_rx_isr_size = uart_rx_size;
    if(sbv_uart_instance->uart_rx_notify_task != NULL)
    {
        sbv_rtos_notify_give_fromISR (sbv_uart_instance->uart_rx_notify_task,
                                      &xHigherPriorityTaskWoken);
        sbv_rtos_port_yield_fromISR (xHigherPriorityTaskWoken);
    }
}

uint8_t *
sbv_uart_stm32f1xx_rcv_data (sbv_uart_instance_t* sbv_uart_instance, uint16_t *size, uint16_t timeout_ms)
{
    uint8_t* rx_buffer_cur_pos;
    uint8_t* rx_buffer_ret_pos;
    uint16_t rx_buffer_size_left;
    uint32_t notify;
    sbv_rtos_tick_type_t tick_to_wait;

    if (! sbv_uart_instance)
        return NULL;

    tick_to_wait = sbv_rtos_ms_to_tick(timeout_ms);

    sbv_rtos_mutex_lock (sbv_uart_instance->mutex);

    sbv_uart_instance->uart_rx_notify_task = sbv_rtos_get_current_task_handle();

    /* Register the UART RX DMA ISR for the first time */
    if (! sbv_uart_instance->rx_isr_registered)
    {
        sbv_uart_instance->uart_rx_buffer->head = 0;
        sbv_uart_instance->uart_rx_buffer->rear = 0;
        sbv_uart_stm32f1xx_rx_idle_deteciton_start (sbv_uart_instance->uart_handle,
                                                    sbv_uart_instance->uart_rx_dma_handle,
                                                    sbv_uart_instance->uart_rx_buffer->buff,
                                                    SBV_UART_RX_BUFFER_SIZE);
        sbv_uart_instance->rx_isr_registered = SBV_TRUE;
    }

    /* Blocking call until timeout */
    notify = sbv_rtos_notify_take(SBV_RTOS_TRUE, tick_to_wait);
    if (notify == 0)
    {
        sbv_rtos_mutex_unlock (sbv_uart_instance->mutex);
        /* No notification is received after the timeout event */
        *size = 0;
        return NULL;
    }

    sbv_uart_instance->uart_rx_notify_task = NULL;

    sbv_uart_instance->uart_rx_buffer->head = (sbv_uart_instance->uart_rx_buffer->head + \
                                                sbv_uart_instance->uart_rx_isr_size) % sbv_uart_instance->uart_rx_buffer->capacity;

    /* Re-initiate the DMA idle line detection interrutp */
    rx_buffer_cur_pos   = sbv_uart_instance->uart_rx_buffer->buff + sbv_uart_instance->uart_rx_buffer->head;
    rx_buffer_size_left = sbv_cqbuff_avail_size (sbv_uart_instance->uart_rx_buffer);

    sbv_uart_stm32f1xx_rx_idle_deteciton_start (sbv_uart_instance->uart_handle,
                                                sbv_uart_instance->uart_rx_dma_handle, 
                                                rx_buffer_cur_pos, rx_buffer_size_left);

    *size = sbv_uart_instance->uart_rx_isr_size;

    sbv_uart_instance->uart_rx_isr_size = 0;
    rx_buffer_ret_pos = sbv_uart_instance->uart_rx_buffer->buff + \
                            sbv_uart_instance->uart_rx_buffer->rear;

    sbv_rtos_mutex_unlock (sbv_uart_instance->mutex);

    if (sbv_uart_instance->uart_rx_cb)
    {
        (*sbv_uart_instance->uart_rx_cb) (rx_buffer_ret_pos, *size);
        /* Update current pointer position */
        sbv_uart_instance->uart_rx_buffer->rear += *size;
    }

    return rx_buffer_ret_pos;
}

int
sbv_uart_stm32f1xx_register_rx_cb (sbv_uart_instance_t* sbv_uart_instance,
                                   int (*uart_rx_cb)(uint8_t *, const uint16_t))
{
    if (! uart_rx_cb || ! sbv_uart_instance)
        return -1;

    sbv_rtos_mutex_lock (sbv_uart_instance->mutex);

    sbv_uart_instance->uart_rx_cb = uart_rx_cb;

    sbv_rtos_mutex_unlock (sbv_uart_instance->mutex);
    return 0;
}
#endif /* STM32F1xx */