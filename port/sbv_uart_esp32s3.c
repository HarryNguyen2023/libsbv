#include <stdlib.h>
#include <string.h>
#include "sbv_rtos.h"
#include "sbv_uart.h"
#include "sbv_uart_esp32s3.h"

#ifdef ESP32xx_IDF
sbv_uart_instance_t     sbv_uart_instance;
sbv_uart_dma_handle_t   uart_rx_intr_handle;

extern sbv_rtos_mutex_t SBV_UART_TX_BUFFER_MUTEX;
extern sbv_rtos_mutex_t SBV_UART_RX_BUFFER_MUTEX;

void sbv_uart_esp32s3_rx_hw_callback(void *arg);

static void
sbv_uart_esp32s3_set_config(sbv_uart_cfg_t *uart_cfg, sbv_uart_baudrate_t baudrate)
{
    if(!uart_cfg)
        return;

    uart_cfg->baud_rate     = baudrate;
    uart_cfg->data_bits     = UART_DATA_8_BITS;
    uart_cfg->parity        = UART_PARITY_DISABLE;
    uart_cfg->stop_bits     = UART_STOP_BITS_1;
    uart_cfg->flow_ctrl     = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg->source_clk    = UART_SCLK_DEFAULT;
}

static void
sbv_uart_esp32s3_instance_init(sbv_uart_handle_t* uart_handle, sbv_uart_dma_handle_t* uart_dma_handle,
                               sbv_uart_baudrate_t baudrate)
{
    /* Initiate the rx instance */
    sbv_uart_instance.uart_rx_buffer         = (uint8_t *)sbv_rtos_malloc(sizeof (uint8_t) * SBV_UART_RX_BUFFER_SIZE);
    if (! sbv_uart_instance.uart_rx_buffer)
    {
        /* LOG */
        goto ERR_EXIT;
    }
    sbv_uart_instance.uart_tx_buffer         = (uint8_t *)sbv_rtos_malloc(sizeof (uint8_t) * SBV_UART_TX_BUFFER_SIZE);
    if (! sbv_uart_instance.uart_tx_buffer)
    {
        /* LOG */
        goto ERR_EXIT;
    }
    sbv_uart_instance.uart_handle            = uart_handle;
    sbv_uart_instance.uart_rx_dma_handle     = uart_dma_handle;

    sbv_uart_instance.uart_baudrate          = baudrate;
    sbv_uart_instance.uart_rx_notify_task    = NULL;
    sbv_uart_instance.uart_rx_buffer_pos     = 0;
    sbv_uart_instance.uart_rx_isr_size       = 0;

    /*Create the mutex for the UART TX and RX FIFO*/
    sbv_rtos_mutex_create(SBV_UART_TX_BUFFER_MUTEX);
    sbv_rtos_mutex_create(SBV_UART_RX_BUFFER_MUTEX);

    /* Initiate the tx and rx buffers */
    memset(sbv_uart_instance.uart_tx_buffer, 0, SBV_UART_TX_BUFFER_SIZE);
    memset(sbv_uart_instance.uart_rx_buffer, 0, SBV_UART_RX_BUFFER_SIZE);

    return;

ERR_EXIT:
    if (sbv_uart_instance.uart_rx_buffer)  sbv_rtos_free (sbv_uart_instance.uart_rx_buffer);
    return;
}

void
sbv_uart_esp32s3_init(sbv_gpio_num_t uart_pin[2], sbv_uart_handle_t* uart_port,
                      sbv_uart_dma_handle_t* uart_intr_handle, sbv_uart_baudrate_t baudrate)
{
    sbv_uart_cfg_t uart_cfg;
    sbv_uart_intr_config_t uart_intr_cfg;

    if (! uart_port || ! uart_intr_handle)
        return;

    sbv_uart_esp32s3_driver_install (*uart_port, SBV_UART_RX_BUFFER_SIZE, SBV_UART_TX_BUFFER_SIZE, 0, NULL, ESP_INTR_FLAG_IRAM);

    memset(&uart_cfg, 0, sizeof (sbv_uart_cfg_t));
    sbv_uart_esp32s3_set_config (&uart_cfg, baudrate);
    sbv_uart_esp32s3_param_config (*uart_port, &uart_cfg);

    sbv_uart_esp32s3_set_pin (*uart_port, uart_pin[0], uart_pin[1], UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    sbv_uart_esp32s3_intr_alloc(ETS_UART1_INTR_SOURCE, 0, sbv_uart_esp32s3_rx_hw_callback, NULL, uart_intr_handle);

    memset (&uart_intr_cfg, 0, sizeof (sbv_uart_intr_config_t));
    uart_intr_cfg.rxfifo_full_thresh    = 1;
    uart_intr_cfg.intr_enable_mask      = (UART_RXFIFO_FULL_INT_ENA_M);

    sbv_uart_esp32s3_intr_config(*uart_port, &uart_intr_cfg);

    sbv_uart_esp32s3_enable_rx_intr(*uart_port);

    sbv_uart_esp32s3_instance_init (uart_port, uart_intr_handle, baudrate);
}

static int
sbv_uart_esp32s3_tx_send_pkt(sbv_uart_handle_t* uart_port, uint8_t* uart_tx_buffer, uint16_t uart_tx_size)
{
    int ret;

    if(! uart_port || ! uart_tx_buffer)
        return SBV_ERROR;

    if (uart_write_bytes(*uart_port, uart_tx_buffer, uart_tx_size) <= 0)
        ret =  SBV_ERROR;
    else
        ret = SBV_OK;

    return ret;
}

int
sbv_uart_esp32s3_tx_packet_format(uint8_t* uart_tx_data, uint16_t uart_tx_size, uint8_t* uart_tx_buffer)
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
sbv_uart_esp32s3_send_data(sbv_uart_handle_t* uart_handle, uint8_t* uart_tx_data, uint16_t uart_tx_size, uint16_t timeout_ms)
{
    int ret = SBV_OK, total_tx_bytes = 0, cur_tx_bytes = 0;

    if(! uart_handle || ! uart_tx_data)
        return 0;

    SBV_UART_TX_BUFFER_MUTEX_LOCK;

    while (total_tx_bytes < uart_tx_size)
    {
        cur_tx_bytes = sbv_uart_esp32s3_tx_packet_format(uart_tx_data + total_tx_bytes,
                                                         (uart_tx_size - total_tx_bytes),
                                                         sbv_uart_instance.uart_tx_buffer);
        ret = sbv_uart_esp32s3_tx_send_pkt(uart_handle, sbv_uart_instance.uart_tx_buffer, cur_tx_bytes);
        if (ret != SBV_OK)
            continue;
        total_tx_bytes += cur_tx_bytes;
    }

    SBV_UART_TX_BUFFER_MUTEX_UNLOCK;

    return total_tx_bytes;
}

void IRAM_ATTR sbv_uart_esp32s3_rx_hw_callback (void *arg)
{
    sbv_rtos_base_type_t xHigherPriorityTaskWoken = SBV_RTOS_FALSE;
    uint16_t uart_rx_size = 0, uart_rx_status = 0;

    if(*(sbv_uart_instance.uart_handle) == UART_NUM_1)
    {
        uart_rx_status  = UART1.int_st.val;
        uart_rx_size    = UART1.status.rxfifo_cnt;
    }
    else if(*(sbv_uart_instance.uart_handle) == UART_NUM_2)
    {
        uart_rx_status  = UART2.int_st.val;
        uart_rx_size    = UART2.status.rxfifo_cnt;
    }

    sbv_uart_instance.uart_rx_isr_size = uart_rx_size;
    /* Copy received data into buffer */
    for (int i = 0; i < uart_rx_size && ((sbv_uart_instance.uart_rx_buffer_pos + i) < SBV_UART_RX_BUFFER_SIZE); i++) 
    {
        if(*(sbv_uart_instance.uart_handle) == UART_NUM_1)
            sbv_uart_rx_buffer[sbv_uart_instance.uart_rx_buffer_pos + i] = UART1.fifo.rxfifo_rd_byte;
        else if(*(sbv_uart_instance.uart_handle) == UART_NUM_1)
            sbv_uart_rx_buffer[sbv_uart_instance.uart_rx_buffer_pos + i] = UART2.fifo.rxfifo_rd_byte;
    }

    /* Clear interrupt status */
    if(uart_rx_status & UART_RXFIFO_FULL_INT_ST_M)
        sbv_uart_esp32s3_clear_intr_status (*(sbv_uart_instance.uart_handle), UART_RXFIFO_FULL_INT_CLR);

    if(sbv_uart_instance.uart_rx_notify_task != NULL)
    {
        sbv_rtos_notify_give_fromISR (sbv_uart_instance.uart_rx_notify_task, &xHigherPriorityTaskWoken);
        sbv_rtos_port_yield_fromISR (xHigherPriorityTaskWoken);
    }
}

uint8_t *
sbv_uart_esp32s3_rcv_data (uint16_t *size, uint16_t timeout_ms)
{
    sbv_rtos_tick_type_t tick_to_wait;

    tick_to_wait = sbv_rtos_ms_to_tick(timeout_ms);

    SBV_UART_RX_BUFFER_MUTEX_LOCK;

    sbv_uart_instance.uart_rx_notify_task = sbv_rtos_get_current_task_handle();
    if((sbv_uart_instance.uart_rx_buffer_pos + \
        sbv_uart_instance.uart_rx_isr_size) >= SBV_UART_RX_BUFFER_SIZE)
    {
        sbv_uart_instance.uart_rx_buffer_pos = SBV_UART_RX_BUFFER_SIZE;
    }
    else
        sbv_uart_instance.uart_rx_buffer_pos += sbv_uart_instance.uart_rx_isr_size;

    sbv_rtos_notify_take(SBV_RTOS_TRUE, tick_to_wait);
    sbv_uart_instance.uart_rx_notify_task = NULL;

    SBV_UART_RX_BUFFER_MUTEX_UNLOCK;

    *size = sbv_uart_instance.uart_rx_buffer_pos;

    sbv_uart_instance.uart_rx_buffer_pos   = 0;
    sbv_uart_instance.uart_rx_isr_size     = 0;
    return sbv_uart_instance.uart_rx_buffer;
}
#endif /* ESP32xx_IDF */