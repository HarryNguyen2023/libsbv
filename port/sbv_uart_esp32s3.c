#include <stdlib.h>
#include <string.h>
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_uart.h"
#include "sbv_uart_esp32s3.h"

#ifdef ESP32xx_IDF
struct sbv_uart_instances_list_t sbv_uart_instances_list = {0};

#define SBV_UART_MUTEX_LOCK(Instance) \
        sbv_rtos_mutex_lock(Instance->mu)

#define SBV_UART_MUTEX_UNLOCK(Instance) \
        sbv_rtos_mutex_unlock(Instance->mu)

void sbv_uart_esp32s3_rx_hw_callback(void *arg);

/* Return the UART instance that owns the given HAL UART handle. */
static sbv_uart_instance_t*
sbv_uart_esp32s3_get_instance_by_handle (sbv_uart_handle_t* uart_handle)
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
sbv_uart_esp32s3_add_instance_to_list (sbv_uart_instance_t *uart_instance)
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
sbv_uart_esp32s3_set_config(sbv_uart_cfg_t *uart_cfg, sbv_uart_baudrate_t baudrate)
{
    if(! uart_cfg)
        return;

    uart_cfg->baud_rate     = baudrate;
    uart_cfg->data_bits     = UART_DATA_8_BITS;
    uart_cfg->parity        = UART_PARITY_DISABLE;
    uart_cfg->stop_bits     = UART_STOP_BITS_1;
    uart_cfg->flow_ctrl     = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg->source_clk    = UART_SCLK_DEFAULT;
}

static int
sbv_uart_esp32s3_instance_init (sbv_uart_instance_t *uart_instance, sbv_uart_handle_t* uart_handle,
                                sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate)
{
    if(! uart_instance || ! uart_handle || ! uart_dma_handle)
        return SBV_ERROR;

    if (sbv_uart_esp32s3_add_instance_to_list (uart_instance) != SBV_OK)
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

    uart_instance->uart_handle            = uart_handle;
    uart_instance->uart_rx_dma_handle     = uart_dma_handle;

    uart_instance->uart_baudrate          = baudrate;
    uart_instance->uart_rx_notify_task    = NULL;

    /* Create the mutex for the UART channel */
    sbv_rtos_mutex_create(uart_instance->mu);

    return SBV_OK;
}

int
sbv_uart_esp32s3_init(sbv_uart_instance_t *uart_instance, sbv_uart_handle_t* uart_handle,
                      sbv_uart_dma_handle_t* uart_dma_handle, sbv_uart_baudrate_t baudrate,
                      sbv_gpio_num_t uart_pin[2])
{
    sbv_uart_cfg_t uart_cfg;
    sbv_uart_intr_config_t uart_intr_cfg;

    if(! uart_instance || ! uart_handle || ! uart_dma_handle)
        return SBV_ERROR;

    sbv_uart_esp32s3_driver_install (*uart_handle, SBV_UART_RX_BUFFER_SIZE,
                                    SBV_UART_TX_BUFFER_SIZE, 0, NULL,
                                    ESP_INTR_FLAG_IRAM);

    memset(&uart_cfg, 0, sizeof (sbv_uart_cfg_t));
    sbv_uart_esp32s3_set_config (&uart_cfg, baudrate);
    sbv_uart_esp32s3_param_config (*uart_handle, &uart_cfg);

    sbv_uart_esp32s3_set_pin (*uart_handle, uart_pin[0], uart_pin[1],
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    sbv_uart_esp32s3_intr_alloc(ETS_UART1_INTR_SOURCE, 0,
                                sbv_uart_esp32s3_rx_hw_callback,
                                NULL, uart_dma_handle);

    memset (&uart_intr_cfg, 0, sizeof (sbv_uart_intr_config_t));
    uart_intr_cfg.rxfifo_full_thresh    = 1;
    uart_intr_cfg.intr_enable_mask      = (UART_RXFIFO_FULL_INT_ENA_M);

    sbv_uart_esp32s3_intr_config(*uart_handle, &uart_intr_cfg);

    sbv_uart_esp32s3_enable_rx_intr(*uart_handle);

    return sbv_uart_esp32s3_instance_init (uart_instance, uart_handle, uart_dma_handle, baudrate);
}

static uint16_t
sbv_uart_esp32s3_tx_send_pkt(sbv_uart_handle_t* uart_port, uint8_t* uart_tx_buffer, uint16_t uart_tx_size)
{
    int ret;

    if(! uart_port || ! uart_tx_buffer)
        return SBV_ERROR;

    return uart_write_bytes(*uart_port, uart_tx_buffer, uart_tx_size);
}

int
sbv_uart_esp32s3_send_data (sbv_uart_instance_t* uart_instance,
                            uint8_t* uart_tx_data,
                            uint16_t uart_tx_size, uint16_t timeout_ms)
{
    uint8_t try_num;
    uint16_t total_tx_len = 0, cur_tx_len = 0;

    if(! uart_instance || ! uart_tx_data || uart_tx_size == 0)
        return 0;

    SBV_UART_MUTEX_LOCK (uart_instance);

    while (total_tx_len < uart_tx_size)
    {
        cur_tx_len = sbv_uart_esp32s3_tx_send_pkt (uart_instance->uart_handle,
                                                   uart_tx_data + total_tx_len,
                                                   uart_tx_size - total_tx_len);
        if (cur_tx_len <= 0)
        {
            // LOG
            if (try_num++ >= SBV_UART_MAX_WRITE_TRY)
            {
                SBV_UART_MUTEX_UNLOCK (uart_instance);
                return total_tx_len;
            }
        }

        total_tx_len += cur_tx_len;
    }
    
    SBV_UART_MUTEX_UNLOCK (uart_instance);

    return total_tx_len;
}

void IRAM_ATTR sbv_uart_esp32s3_rx_hw_callback (void *arg)
{
    sbv_uart_instance_t *uart_instance = NULL;
    sbv_rtos_base_type_t xHigherPriorityTaskWoken = SBV_RTOS_FALSE;
    uint16_t uart_rx_size = 0, rx_buffer_size_left;
    sbv_uart_handle_t uart_handle = (uart_port_t)(uintptr_t)arg;

    uint32_t intr_status = uart_hal_get_intsts_mask(&uart_context[uart_handle].hal);

    if (intr_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT))
    {
        uart_instance = sbv_uart_esp32s3_get_instance_by_handle (uart_handle);
        if (! uart_instance)
        {
            // LOG
            goto ISR_CLEAR;
        }
        uart_hal_get_rxfifo_len(&uart_context[uart_handle].hal, &uart_rx_size);
        if (uart_rx_size == 0)
            goto ISR_CLEAR;

        rx_buffer_size_left = sbv_cqbuff_avail_size (uart_instance->uart_rx_buffer);
        uart_rx_size = (uart_rx_size < rx_buffer_size_left) \
                         ? uart_rx_size : rx_buffer_size_left;

        uart_instance->uart_rx_buffer->head = (uart_instance->uart_rx_buffer->head == -1) \
                                                ? 0: uart_instance->uart_rx_buffer->head;

        uart_instance->uart_rx_buffer->head = (uart_instance->uart_rx_buffer->head + \
                                                uart_rx_size) % uart_instance->uart_rx_buffer->capacity;

        uart_hal_read_rxfifo (&uart_context[uart_handle].hal,
                              uart_instance->uart_rx_buffer->buff,
                              &uart_rx_size);

ISR_CLEAR:
        /* Clear interrupt status */
        uart_hal_clr_intsts_mask (&uart_context[uart_num].hal,
                                 UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    }

    if(uart_instance->uart_rx_notify_task != NULL)
    {
        sbv_rtos_notify_give_fromISR (uart_instance->uart_rx_notify_task,
                                      &xHigherPriorityTaskWoken);
        sbv_rtos_port_yield_fromISR (xHigherPriorityTaskWoken);
    }
}

int
sbv_uart_esp32s3_rcv_data (sbv_uart_instance_t* uart_instance,
                           uint8_t recv_buff[], uint16_t size,
                           uint16_t timeout_ms)
{
    uint8_t* rx_buffer_ret_pos;
    uint16_t rx_buffer_size;
    sbv_rtos_tick_type_t tick_to_wait;

    if (! uart_instance)
        return SBV_ERROR;

    tick_to_wait = sbv_rtos_ms_to_tick(timeout_ms);

    SBV_UART_MUTEX_LOCK (uart_instance);

    uart_instance->uart_rx_notify_task = sbv_rtos_get_current_task_handle();

    /* Blocking call until timeout */
    sbv_rtos_notify_take(SBV_RTOS_TRUE, tick_to_wait);

    uart_instance->uart_rx_notify_task = NULL;

    uart_instance->uart_rx_buffer->rear = (uart_instance->uart_rx_buffer->rear == -1) \
                                                ? 0: uart_instance->uart_rx_buffer->rear;

    rx_buffer_ret_pos = uart_instance->uart_rx_buffer->buff + \
                            uart_instance->uart_rx_buffer->rear;

    rx_buffer_size = sbv_cqbuff_get_size (uart_instance->uart_rx_buffer);
    if (rx_buffer_size == 0)
    {
        SBV_UART_MUTEX_UNLOCK (uart_instance);
        return 0;
    }

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
            rx_buffer_size = sbv_cqbuff_read (uart_instance->uart_rx_buffer,
                                              recv_buff, rx_buffer_size);
        }
    }

    SBV_UART_MUTEX_UNLOCK (uart_instance);

    return rx_buffer_size;
}
#endif /* ESP32xx_IDF */