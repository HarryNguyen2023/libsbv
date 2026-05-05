#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_task.h"
#include "sbv_can.h"
#include "sbv_uart.h"
#include "sbv_debug.h"
#include "sbv_ota.h"
#include "sbv_ota_msg.h"
#include "sbv_pid.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"
#include "sbv_control_speed.h"
#include "sbv_control_balance.h"

extern sbv_control_balance_t sbv_control_balance;

extern sbv_i2c_handle_t      hi2c1;
extern sbv_uart_handle_t     huart1;
extern sbv_uart_dma_handle_t hdma_usart1_rx;
extern sbv_can_handle_t      hcan;
sbv_gpio_num_t uart_pin[2] = {SBV_GPIO_NUM_13, SBV_GPIO_NUM_7};

// sbv_rtos_task_handle_t sbv_debug_handle;
sbv_rtos_task_handle_t sbv_uart_rx_handle;
sbv_rtos_task_handle_t sbv_balance_ctrl_handle;

void
sbv_init(void)
{
    /* Built-in LED initialization */
    sbv_gpio_init(SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, SBV_GPIO_MODE_OUTPUT);

    /* UART serial initialization */
    sbv_uart_init(uart_pin, &huart1, &hdma_usart1_rx, SBV_UART_BAUDRATE_115200);

    /* CAN interface initialization */
    // sbv_can_init(&hcan);

    /* Initialize the robot control system */
    sbv_control_balance_init(&sbv_control_balance, &hi2c1);

    sbv_ota_update_init();

    sbv_task_init();
}

void
sbv_task_init(void)
{
    // sbv_rtos_task_create(sbv_task_debug_console_task, "serial_tx", 128, NULL, 1, &sbv_debug_handle);

    sbv_rtos_task_create(sbv_task_uart_rx_task, "uart_rx", 256, NULL, 2, &sbv_uart_rx_handle);

    sbv_rtos_task_create(sbv_task_balance_control, "balance_ctrl", 512, NULL, 4, &sbv_balance_ctrl_handle);

    // sbv_rtos_start_task_scheduler();
}

/*
 * Task for testing speed + twist control of the robot
*/
void
sbv_task_balance_control(void *param)
{
    uint32_t start_tick;
    uint16_t timeout_ms;
    sbv_rtos_tick_type_t tick_to_wait;

    start_tick   = sbv_rtos_get_tick();
    timeout_ms   = sbv_control_balance.balance_pid.sampling_time_ms;
    tick_to_wait = sbv_rtos_ms_to_tick(sbv_control_balance.sbv_control_speed.steering_pid.sampling_time_ms);
    for(;;)
    {
        while (sbv_rtos_get_tick() - start_tick < sbv_rtos_ms_to_tick(timeout_ms))
        {
            sbv_control_robot_speed_twist_update(&(sbv_control_balance.sbv_control_speed));
            sbv_rtos_task_delay(tick_to_wait);
        }
        sbv_control_balance_update(&sbv_control_balance);
        start_tick = sbv_rtos_get_tick();
    }
}

// static void
// sbv_task_print_rx(uint8_t *data, uint8_t len)
// {
//     uint16_t timeout_ms = 100;
//     sbv_uart_tx_send_data((uint8_t *)"Received: ", 10, timeout_ms);
//     sbv_uart_tx_send_data(data, len, timeout_ms);
//     sbv_uart_tx_send_data((uint8_t *)"\r\n", 2, timeout_ms);
// }


// /*
//  * Task for transmitting debugging information via UART
//  */
// void
// sbv_task_debug_console_task(void *param)
// {
//     sbv_rtos_tick_type_t tick_to_wait = sbv_rtos_ms_to_tick(100);

//     for(;;)
//     {
//         sbv_debug_tx_logging();

//         sbv_rtos_task_delay(tick_to_wait);
//     }
// }

// int
// sbv_task_uart_rx_cb (uint8_t *data, const uint16_t size)
// {
//     if (! data || ! size)
//         return -1;

//     sbv_task_print_rx (data, size);
//     sbv_debug_command_handle((char *)data, size);

//     return 0;
// }

/*
 * Task for debugging UART rx using interrupt and RTOS direct task notification
 */
void
sbv_task_uart_rx_task(void *param)
{
    uint16_t rcv_data_size  = 0;
    uint16_t timeout_ms     = 100;

    /* Register callback function to handle UART rx */
    sbv_uart_register_rx_cb (sbv_ota_msg_rx_handle);
    for(;;)
    {
        sbv_uart_rx_rcv_data (&rcv_data_size, timeout_ms);
    }
}

// static void
// sbv_task_can_tx_data_fill(uint8_t *can_data, uint8_t num)
// {
//     uint8_t i;

//     for(i = 0; i < SBV_CAN_DATA_MAX_SIZE; ++i)
//         *(can_data + i) = num + i;
// }

// /*
//  * Task for debugging RTOS implementation of CAN TX
// */
// void
// sbv_task_can_tx_debug_task(void *param)
// {
//     uint8_t num = 65;
//     uint8_t can_tx_data[SBV_CAN_DATA_MAX_SIZE];
//     sbv_rtos_tick_type_t tick_to_wait = sbv_rtos_ms_to_tick(150);

//     // sbv_rtos_task_delay(sbv_rtos_ms_to_tick(4000));

//     for(;;)
//     {
//         if(num > 122)
//             num = 65;
//         else
//             num += 8;

//         sbv_task_can_tx_data_fill(can_tx_data, num);

//         sbv_can_send_data(SBV_CAN_MSG_LOGGING, can_tx_data, SBV_CAN_DATA_MAX_SIZE);

//         sbv_rtos_task_delay(tick_to_wait);
//     }
// }
// /*
//  * Task for debugging RTOS implementation of CAN RX
// */
// void sbv_task_can_rx_debug_task(void *param)
// {
//     uint8_t* can_rx_data = NULL;
//     uint8_t  can_rx_len = 0;
//     uint16_t pkt_id;

//     for(;;)
//     {
//         can_rx_data = sbv_can_rcv_data(&can_rx_len, &pkt_id);
//         if(can_rx_data && can_rx_len)
//             sbv_task_print_rx(can_rx_data, can_rx_len);

//         can_rx_len  = 0;
//         can_rx_data = NULL;
//     }
// }

// /*
//  * Task for handling packet received via CAN
// */
// void sbv_task_can_rx_handle(void *param)
// {
//     uint8_t* can_rx_data = NULL;
//     uint8_t can_pkt_len = 0;
//     uint16_t can_pkt_id;

//     for(;;)
//     {
//         can_rx_data = sbv_can_rcv_data(&can_pkt_len, &can_pkt_id);
//         if(can_rx_data && can_pkt_len)
//         {
//             sbv_can_pkt_process(can_rx_data, can_pkt_len, can_pkt_id);
//             sbv_task_print_rx(can_rx_data, can_pkt_len);
//         }
//     }
// }