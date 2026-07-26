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

/************************ Gloabal variables declaration ***************************/
/* Static task objects */
#define STACK_SIZE_BASE 256

static StackType_t uart_rx_stack[STACK_SIZE_BASE];
static StackType_t balance_crtl_stack[STACK_SIZE_BASE * 2];

// sbv_rtos_task_handle_t sbv_debug_handle;
sbv_rtos_task_handle_t sbv_uart_rx_handle;
sbv_rtos_task_handle_t sbv_balance_ctrl_handle;

/* Control obbjects */
sbv_control_balance_t   sbv_control_balance;
sbv_imu_instance_t      sbv_imu_instance;
sbv_i2c_instance_t      sbv_i2c_1;
extern sbv_i2c_handle_t hi2c1;


extern sbv_uart_handle_t     huart1;
extern sbv_uart_dma_handle_t hdma_usart1_rx;
extern sbv_can_handle_t      hcan;
sbv_gpio_num_t uart_pin[2] = {SBV_GPIO_NUM_13, SBV_GPIO_NUM_7};



sbv_uart_instance_t *sbv_uart_1 = NULL;



void
sbv_init(void)
{
    /* Built-in LED initialization */
    sbv_gpio_init(SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, SBV_GPIO_MODE_OUTPUT);

    /* UART serial initialization */
    sbv_uart_1 = sbv_uart_init(uart_pin, &huart1, &hdma_usart1_rx, SBV_UART_BAUDRATE_115200);

    /* CAN interface initialization */
    // sbv_can_init(&hcan);

    /* Initialize the robot control system */
    sbv_control_balance_init(&sbv_control_balance, &sbv_imu_instance, &sbv_i2c_1, &hi2c1);

    sbv_ota_update_init();

    sbv_task_init();
}

void
sbv_task_init(void)
{
    // sbv_rtos_task_create(sbv_task_debug_console_task, "serial_tx", 128, NULL, 1, &sbv_debug_handle);

    sbv_rtos_task_create(sbv_task_uart_rx_task, "uart_rx", 256,
                        NULL, 2, uart_rx_stack, &sbv_uart_rx_handle);

    sbv_rtos_task_create(sbv_task_balance_control, "balance_ctrl", 512,
                        NULL, 4, balance_crtl_stack, &sbv_balance_ctrl_handle);

    // sbv_rtos_start_task_scheduler();
}

/*
 * Task for control of the robot
 */
void
sbv_task_balance_control(void *param)
{
    uint32_t start_tick;
    uint16_t balance_update_period_ms;
    sbv_rtos_tick_type_t speed_update_delay_ticks;

    /* Record the initial tick so the control loop can run at a fixed sample interval. */
    start_tick = sbv_rtos_get_tick();
    /* Use the balance controller sampling period as the main update interval. */
    balance_update_period_ms = sbv_control_balance.balance_pid.sampling_time_ms;
    /* Delay between speed/twist updates follows the steering PID sampling period. */
    speed_update_delay_ticks = sbv_rtos_ms_to_tick(sbv_control_balance.sbv_control_speed.steering_pid.sampling_time_ms);

    for(;;)
    {
        /* Run the speed/twist controller repeatedly until the sampling window expires. */
        while (sbv_rtos_get_tick() - start_tick < sbv_rtos_ms_to_tick(balance_update_period_ms))
        {
            sbv_control_robot_speed_twist_update(&(sbv_control_balance.sbv_control_speed));
            sbv_rtos_task_delay(speed_update_delay_ticks);
        }
        /* Once the interval has elapsed, update balance control and restart the timing window. */
        sbv_control_balance_update(&sbv_control_balance);
        start_tick = sbv_rtos_get_tick();
    }
}

/*
 * Task for debugging UART rx using interrupt and RTOS direct task notification
 */
void
sbv_task_uart_rx_task(void *param)
{
    uint16_t rcv_data_size  = 0;
    uint16_t timeout_ms     = 100;

    /* Register callback function to handle UART rx */
    sbv_uart_register_rx_cb (sbv_uart_1, sbv_ota_msg_rx_handle);
    for(;;)
    {
        sbv_uart_rx_rcv_data (sbv_uart_1, &rcv_data_size, timeout_ms);
    }
}