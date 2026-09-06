#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "sbv.h"
#include "sbv_uart.h"

extern sbv_uart_instance_t sbv_uart_1;

#define SBV_LOG_BUFFER_SIZE  256
#define SBV_LOG_TIMEOUT_MS   100

static char buffer[SBV_LOG_BUFFER_SIZE];

void sbv_log_printf(const char *format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0)
    {
        if (len >= SBV_LOG_BUFFER_SIZE)
            len = SBV_LOG_BUFFER_SIZE - 1;

        sbv_uart_tx_send_data (&sbv_uart_1, (uint8_t*)buffer, len, SBV_LOG_TIMEOUT_MS);
    }
}