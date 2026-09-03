#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_cqbuff.h"
#include "sbv_task.h"
#include "sbv_can.h"
#include "sbv_uart.h"
#include "sbv_debug.h"
#include "sbv_ota_common.h"
#include "sbv_ota.h"
#include "sbv_ota_msg.h"
#include "sbv_ota_slave_fsm.h"
#include "sbv_ota_master_fsm.h"
#include "sbv_pid.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"
#include "sbv_control_speed.h"
#include "sbv_control_balance.h"

/************************ Gloabal variables declaration ***************************/
/* Static task objects */
static sbv_rtos_stack_type_t debug_stack[STACK_SIZE_BASE];
static sbv_rtos_stack_type_t balance_crtl_stack[STACK_SIZE_BASE * 4];

sbv_rtos_task_handle_t sbv_debug_handle;
sbv_rtos_task_handle_t sbv_balance_ctrl_handle;

/* Control task's obbjects */
sbv_control_balance_t   sbv_control_balance;
sbv_imu_instance_t      sbv_imu_instance;
sbv_i2c_instance_t      sbv_i2c_1;
extern sbv_i2c_handle_t hi2c1;

/* Debug task's objects */
extern sbv_uart_handle_t     huart1;
extern sbv_uart_dma_handle_t hdma_usart1_rx;
sbv_gpio_num_t               uart_pin[2] = {SBV_GPIO_NUM_13, SBV_GPIO_NUM_7};
sbv_uart_instance_t          sbv_uart_1;

/* CAN router task's object */
extern sbv_can_handle_t     hcan;
sbv_can_instance_t          sbv_can_instance;

sbv_ota_ipc_t   sbv_ota_queues;

void sbv_led_init_blink (void);
void sbv_task_init(void);
void sbv_task_balance_control(void *param);
void sbv_task_debug_console_task(void *param);
void sbv_task_ota_init (uint8_t is_master);

void
sbv_init(void)
{
    /* Built-in LED initialization */
    sbv_gpio_init(SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, SBV_GPIO_MODE_OUTPUT);

    /* UART serial initialization */
    sbv_uart_init(&sbv_uart_1, &huart1, &hdma_usart1_rx, SBV_UART_BAUDRATE_115200, uart_pin);

    /* CAN interface initialization */
    sbv_can_init(&sbv_can_instance, &hcan);

    /* Initialize the robot control system */
    sbv_control_balance_init(&sbv_control_balance, &sbv_imu_instance, &sbv_i2c_1, &hi2c1);

    /* Blink LED and wait for hardware system to stablize before starting software tasks */
    sbv_led_init_blink();

    sbv_task_ota_init (SBV_FALSE);

    sbv_task_init();
}

void
sbv_led_init_blink (void)
{
    uint16_t blink_time_ms          = 5000;
    uint16_t blink_toggle_time_ms   = 100;

    for (uint8_t i = 0; i < (blink_time_ms / blink_toggle_time_ms); ++i)
        sbv_gpio_toggle_pin (SBV_GPIO_BUILT_IN_LED_TYPE, SBV_GPIO_BUILT_IN_LED, blink_toggle_time_ms);
}

void
sbv_task_init(void)
{
    sbv_rtos_task_create(sbv_task_debug_console_task, "debug", STACK_SIZE_BASE,
                        NULL, 2, debug_stack, &sbv_debug_handle);

    sbv_rtos_task_create(sbv_task_balance_control, "balance_ctrl", STACK_SIZE_BASE * 4,
                        NULL, 4, balance_crtl_stack, &sbv_balance_ctrl_handle);

    sbv_rtos_start_task_scheduler();
}

/*
 * Task for control of the robot
 */
void
sbv_task_balance_control(void *param)
{
    sbv_rtos_tick_type_t balance_update_period_ticks, balance_deadline_tick;
    sbv_rtos_tick_type_t speed_update_delay_ticks, speed_wake_tick;

    /* Use the balance controller sampling period as the main update interval. */
    balance_update_period_ticks = sbv_rtos_ms_to_tick(sbv_control_balance_get_balance_sampling_time_ms(&sbv_control_balance));
    /* Delay between speed/twist updates follows the steering PID sampling period. */
    speed_update_delay_ticks = sbv_rtos_ms_to_tick(sbv_control_balance_get_speed_sampling_time_ms(&sbv_control_balance));

    for(;;)
    {
        /* 
         * Outer slower PID balance control loop: sample IMU and held balance term,
         * then feedforward the control value into inner faster speed and twist
         * control loop to apply the balance held term in every actuator update
         */
        sbv_control_balance_update(&sbv_control_balance);

        balance_deadline_tick = sbv_rtos_get_tick() + balance_update_period_ticks;
        speed_wake_tick = sbv_rtos_get_tick();

        /* Run the speed/twist controller on a fixed cadence until the balance window expires. */
        while (sbv_rtos_get_tick() < balance_deadline_tick)
        {
            sbv_control_balance_update_speed_twist_control (&sbv_control_balance);
            sbv_rtos_task_delay_until(&speed_wake_tick, speed_update_delay_ticks);
        }
    }
}

/*
 * Task for debugging via UART
 */
void
sbv_task_debug_console_task(void *param)
{
    uint32_t start_tick;
    uint16_t debug_streaming_delay_ms = 500;
    uint8_t uart_rcv_sampling_ms      = 10;
    uint8_t uart_rx_timeout_ms        = 90;

    /* Record the initial tick so the control loop can run at a fixed sample interval. */
    start_tick = sbv_rtos_get_tick();
    /* Register callback function to handle UART rx */
    sbv_debug_set_uart_interface (&sbv_uart_1);

    for(;;)
    {
        // Check for UART reception new message
        while (sbv_rtos_get_tick() - start_tick < sbv_rtos_ms_to_tick(debug_streaming_delay_ms))
        {
            sbv_uart_rx_rcv_data (&sbv_uart_1, NULL, 0, uart_rx_timeout_ms);

            sbv_rtos_task_delay (sbv_rtos_ms_to_tick (uart_rcv_sampling_ms));
        }

        // Send logging data over UART to console for debugging
        sbv_debug_tx_logging ();
        start_tick = sbv_rtos_get_tick();
    }
}

void
sbv_task_ota_init (uint8_t is_master) {
    int ret;

    ret = sbv_ota_ipc_queue_init (&sbv_ota_queues);
    if (ret != SBV_OK) {
        // LOG
        return;
    }

    sbv_ota_update_init (&sbv_ota_queues);
    sbv_ota_slave_fsm_init (&sbv_ota_queues);

    if (is_master) {
        sbv_ota_master_fsm_init ();
    }
}