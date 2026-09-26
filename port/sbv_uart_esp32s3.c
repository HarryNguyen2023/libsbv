#include <stdlib.h>
#include <string.h>

#include "sbv.h"
#include "sbv_log.h"
#include "sbv_rtos.h"
#include "sbv_gpio.h"
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
        LOG_ERROR ("UART instance reach limit, maxmium allowable is %u", SBV_UART_MAX_CHANNEL);
        return SBV_ERROR;
    }

    /* Initiate the rx instance */
    uart_instance->uart_rx_cb             = NULL;

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

    if(! uart_instance || ! uart_handle) {
        LOG_ERROR ("Invalid input, skip UART port initialization");
        return SBV_ERROR;
    }

    memset(&uart_cfg, 0, sizeof (sbv_uart_cfg_t));
    sbv_uart_esp32s3_set_config (&uart_cfg, baudrate);
    sbv_uart_esp32s3_param_config (*uart_handle, &uart_cfg);

    sbv_uart_esp32s3_set_pin (*uart_handle, uart_pin[0], uart_pin[1],
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    sbv_uart_esp32s3_driver_install (*uart_handle, SBV_UART_RX_BUFFER_SIZE,
                                    SBV_UART_TX_BUFFER_SIZE, 0, NULL, 0);

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
sbv_uart_esp32s3_send_data (sbv_uart_instance_t* uart_instance, uint8_t* uart_tx_data,
                            uint16_t uart_tx_size, uint16_t timeout_ms)
{
    uint8_t try_num = 0;
    uint16_t total_tx_len = 0;
    int cur_tx_len = 0;

    if(! uart_instance || ! uart_tx_data || uart_tx_size == 0) {
        LOG_ERROR ("Invalid input, skip sending UART data");
        return 0;
    }

    // SBV_UART_MUTEX_LOCK (uart_instance);

    while (total_tx_len < uart_tx_size)
    {
        cur_tx_len = sbv_uart_esp32s3_tx_send_pkt (uart_instance->uart_handle,
                                                   uart_tx_data + total_tx_len,
                                                   uart_tx_size - total_tx_len);
        if (cur_tx_len <= 0)
        {
            LOG_ERROR ("Fail to send UART data on port %u, retry num=%d",
                       *(uart_instance->uart_handle), ++try_num);

            if (try_num >= SBV_UART_MAX_WRITE_TRY)
            {
                // SBV_UART_MUTEX_UNLOCK (uart_instance);
                return total_tx_len;
            }

            continue;
        }

        total_tx_len += cur_tx_len;
    }
    
    // SBV_UART_MUTEX_UNLOCK (uart_instance);

    return total_tx_len;
}

int
sbv_uart_esp32s3_rcv_data (sbv_uart_instance_t* uart_instance,
                           uint8_t recv_buff[], uint16_t size,
                           uint16_t timeout_ms)
{
    int read_bytes;
    sbv_rtos_tick_type_t tick_to_wait;

    if (! uart_instance || ! recv_buff || size == 0) {
        LOG_ERROR ("Invalid input, skip receiving UART data");
        return 0;
    }

    tick_to_wait = sbv_rtos_ms_to_tick(timeout_ms);

    SBV_UART_MUTEX_LOCK (uart_instance);

    read_bytes = uart_read_bytes (*(uart_instance->uart_handle), recv_buff, size, tick_to_wait);

    // Invoke UART cb if exists
    if (read_bytes > 0 && uart_instance->uart_rx_cb) {
        LOG_DEBUG ("Invoking UART receive cb with %u bytes", read_bytes);

        (*uart_instance->uart_rx_cb) (recv_buff, read_bytes);
    }

    SBV_UART_MUTEX_UNLOCK (uart_instance);

    LOG_DEBUG ("Receive %u bytes over UART port %u", read_bytes, *(uart_instance->uart_handle));

    return read_bytes;
}

int
sbv_uart_esp32s3_register_rx_cb (sbv_uart_instance_t* uart_instance,
                                 int (*uart_rx_cb)(uint8_t *, const uint16_t))
{
    if (! uart_rx_cb || ! uart_instance) {
        LOG_ERROR ("Invalid input, failed to register UART receive cb");
        return -1;
    }

    SBV_UART_MUTEX_LOCK (uart_instance);

    uart_instance->uart_rx_cb = uart_rx_cb;

    SBV_UART_MUTEX_UNLOCK (uart_instance);
    return SBV_OK;
}
#endif /* ESP32xx_IDF */