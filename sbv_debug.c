#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sbv.h"
#include "sbv_uart.h"
#include "sbv_imu.h"
#include "sbv_gpio.h"
#include "sbv_motor.h"
#include "sbv_pid.h"
#include "sbv_control_balance.h"
#include "sbv_debug.h"

#define SBV_DEBUG_TX_TIMEOUT_MS     (5)
#define SBV_DEBUG_COMMAND_MAX_LEN   (22)
#define SBV_DEBUG_MAX_COMMAND_SLOT  (7)
#define SBV_DEBUG_MAX_LEN_PER_SLOT  (4)
#define SBV_DEBUG_TX_MAX_DATA_LEN   (100)

char rcv_command[SBV_DEBUG_MAX_COMMAND_SLOT][SBV_DEBUG_MAX_LEN_PER_SLOT + 1];
sbv_debug_tx_t sbv_tx_debug_command = SBV_DEBUG_TX_NONE;
char tx_logging_buffer[SBV_DEBUG_TX_MAX_DATA_LEN];
char rx_command_buffer[SBV_DEBUG_COMMAND_MAX_LEN + 1];

extern sbv_control_balance_t sbv_control_balance;

struct sbv_debug_interface_t {
    sbv_uart_instance_t *uart_instance;
};

struct sbv_debug_interface_t sbv_debug_interface = {0};

uint8_t
sbv_debug_set_uart_interface (sbv_uart_instance_t *uart_instance)
{
    if (! uart_instance)
        return -1;

    sbv_debug_interface.uart_instance = uart_instance;

    sbv_uart_register_rx_cb (uart_instance, sbv_debug_command_handle);
    return 0;
}

static void
sbv_debug_set_command_handle(void);
static void
sbv_debug_get_command_handle(void);
static void
sbv_debug_reset_command_handle(void);

/*
 * Strip trailing characters introduces by console applications
 */
static uint16_t
sbv_debug_strip_line_ending (char *buffer, uint16_t len)
{
    while (len > 0)
    {
        char trailing = buffer[len - 1];

        if (trailing == '\r' || trailing == '\n'
            || trailing == ' ' || trailing == '\t')
        {
            buffer[--len] = '\0';
            continue;
        }

        break;
    }

    return len;
}

static int
sbv_debug_break_command(char* rcv_buffer, uint16_t size)
{
    uint8_t i = 0, max_idx;
    char *token;
    char *saveptr;

    if(!rcv_buffer || size == 0)
        return -1;

    max_idx = sizeof(rcv_command) / sizeof(rcv_command[0]);
    memset(rcv_command, 0, sizeof(rcv_command));

    token = strtok_r(rcv_buffer, " ", &saveptr);

    while (token != NULL) 
    {
        if (strlen(token) > SBV_DEBUG_MAX_LEN_PER_SLOT)
        {
            // LOG
            return -1;
        }
        strcpy(rcv_command[i], token);
        rcv_command[i][SBV_DEBUG_MAX_LEN_PER_SLOT] = '\0';
        token = strtok_r(NULL, " ", &saveptr);

        /* Avoid buffer overflow */
        if(++i == max_idx)
            break;
    }

    return SBV_OK;
}

int
sbv_debug_command_handle(char* rcv_buffer, const uint16_t size)
{
    uint8_t first_command, command_size;

    if(!rcv_buffer || size == 0)
        return -1;

    command_size = (size < SBV_DEBUG_COMMAND_MAX_LEN) ? size : SBV_DEBUG_COMMAND_MAX_LEN;
    memcpy(rx_command_buffer, rcv_buffer, command_size);
    rx_command_buffer[SBV_DEBUG_COMMAND_MAX_LEN] = '\0';

    command_size = sbv_debug_strip_line_ending(rx_command_buffer, command_size);
    if (command_size == 0)
        return -1;

    if (sbv_debug_break_command(rx_command_buffer, command_size) != SBV_OK)
    {
        sbv_tx_debug_command = SBV_DEBUG_TX_NONE;
        return -1;
    }

    if((strlen(rcv_command[0]) != 1) || (strlen(rcv_command[1]) != 1))
    {
        sbv_tx_debug_command = SBV_DEBUG_TX_NONE;
        return -1;
    }

    first_command = rcv_command[0][0];

    switch (first_command)
    {
    case SBV_SET_VALUE:
        sbv_debug_set_command_handle();
        break;

    case SBV_GET_VALUE:
        sbv_debug_get_command_handle();
        break;

    case SBV_RESET_VALUE:
        sbv_debug_reset_command_handle();
        break;

    default:
        break;
    }

    return 0;
}

static void
sbv_debug_set_pid_gain(char pid)
{
    char motor_name;
    float pid_gains[3];
    uint8_t i;

    switch (pid)
    {
    case SBV_STEERING_PID:
        /* Get 3 of the gains */
        for(i = 0; i < 3; ++i)
            pid_gains[i] = strtof(rcv_command[3 + i], NULL);

        sbv_control_balance_set_steering_pid_gain(&(sbv_control_balance),
                                                  pid_gains[0], pid_gains[1], pid_gains[2]);
        break;

    case SBV_BALANCE_PID:
        /* Get 3 of the gains */
        for(i = 0; i < 3; ++i)
            pid_gains[i] = strtof(rcv_command[3 + i], NULL);

        sbv_control_balance_set_balance_pid_gain (&sbv_control_balance,
                                                  pid_gains[0], pid_gains[1], pid_gains[2]);
        break;

    case SBV_SPEED_PID:
        /* Get the name of motor */
        motor_name = rcv_command[3][0];

        /* Get 3 of the gains */
        for(i = 0; i < 3; ++i)
            pid_gains[i] = strtof(rcv_command[4 + i], NULL);

        if(motor_name == SBV_DEBUG_MOTOR_LEFT)
            sbv_control_balance_set_speed_pid_gain(&(sbv_control_balance), SBV_MOTOR_LEFT,
                                                   pid_gains[0], pid_gains[1], pid_gains[2]);
        else if(motor_name == SBV_DEBUG_MOTOR_RIGHT)
            sbv_control_balance_set_speed_pid_gain(&(sbv_control_balance), SBV_MOTOR_RIGHT,
                                                   pid_gains[0], pid_gains[1], pid_gains[2]);
        break;

    default:
        break;
    }
}

static void
sbv_debug_set_pid_target(char pid)
{
    char motor_name;
    float pid_target;

    switch (pid)
    {
    case SBV_STEERING_PID:
        pid_target = strtof(rcv_command[3], NULL);

        sbv_control_balance_set_robot_twist_target(&(sbv_control_balance), pid_target);
        break;

    case SBV_BALANCE_PID:
        pid_target = strtof(rcv_command[3], NULL);

        sbv_control_balance_set_balance_control_target(&(sbv_control_balance), pid_target);
        break;

    case SBV_SPEED_PID:
        /* Get the name of motor */
        motor_name = rcv_command[3][0];

        pid_target = strtof(rcv_command[4], NULL);

        if(motor_name == SBV_DEBUG_MOTOR_LEFT)
            sbv_control_balance_set_motor_speed_target(&(sbv_control_balance), SBV_MOTOR_LEFT, pid_target);
        else if(motor_name == SBV_DEBUG_MOTOR_RIGHT)
            sbv_control_balance_set_motor_speed_target(&(sbv_control_balance), SBV_MOTOR_RIGHT, pid_target);
        break;

    default:
        break;
    }
}

static void
sbv_debug_set_command_handle(void)
{
    char second_command, third_command;

    if((strlen(rcv_command[2]) != 1))
        return;

    second_command  = rcv_command[1][0];
    third_command   = rcv_command[2][0];

    switch (second_command)
    {
    case SBV_PID_GAIN:
        sbv_debug_set_pid_gain(third_command);
        break;

    case SBV_PID_TARGET:
        sbv_debug_set_pid_target(third_command);
        break;

    default:
        break;
    }
}

static void
sbv_debug_get_pid_command_handle(char pid)
{
    char motor_name;

    switch (pid)
    {
    case SBV_STEERING_PID:
        sbv_tx_debug_command = SBV_DEBUG_TX_STEERING_PID;
        break;

    case SBV_BALANCE_PID:
        sbv_tx_debug_command = SBV_DEBUG_TX_BALANCE_PID;
        break;

    case SBV_SPEED_PID:
        motor_name = rcv_command[3][0];
        if(motor_name == SBV_DEBUG_MOTOR_LEFT)
            sbv_tx_debug_command = SBV_DEBUG_TX_SPEED_PID_LEFT;
        else if(motor_name == SBV_DEBUG_MOTOR_RIGHT)
            sbv_tx_debug_command = SBV_DEBUG_TX_SPEED_PID_RIGHT;
        break;

    default:
        sbv_tx_debug_command = SBV_DEBUG_TX_NONE;
        break;
    }
}

static void
sbv_debug_get_command_handle(void)
{
    char second_command, third_command;

    second_command = rcv_command[1][0]; 

    switch (second_command)
    {
    case SBV_ENCODER:
        sbv_tx_debug_command = SBV_DEBUG_TX_ENCODER;
        break;

    case SBV_PID_GAIN:
        third_command = rcv_command[2][0];
        sbv_debug_get_pid_command_handle(third_command);
        break;

    case SBV_IMU:
        sbv_tx_debug_command = SBV_DEBUG_TX_IMU;
        break;

    default:
        sbv_tx_debug_command = SBV_DEBUG_TX_NONE;
        break;
    }
}

static void
sbv_debug_reset_pid_command_handle(char pid)
{
    char motor_name;

    switch (pid)
    {
    case SBV_STEERING_PID:
        sbv_control_balance_reset_steering_pid(&(sbv_control_balance));
        break;

    case SBV_BALANCE_PID:
        sbv_control_balance_reset_balance_pid(&(sbv_control_balance));
        break;

    case SBV_SPEED_PID:
        motor_name = rcv_command[3][0];
        if(motor_name == SBV_DEBUG_MOTOR_LEFT)
            sbv_control_balance_reset_speed_pid(&(sbv_control_balance), SBV_MOTOR_LEFT);
        else if(motor_name == SBV_DEBUG_MOTOR_RIGHT)
            sbv_control_balance_reset_speed_pid(&(sbv_control_balance), SBV_MOTOR_RIGHT);
        break;

    default:
        break;
    }
}

static void
sbv_debug_reset_command_handle(void)
{
    char second_command, third_command;

    second_command = rcv_command[1][0]; 

    switch (second_command)
    {
    case SBV_ENCODER:
        sbv_control_balance_reset_encoder (&(sbv_control_balance));
        break;

    case SBV_PID_GAIN:
        third_command = rcv_command[2][0];
        sbv_debug_reset_pid_command_handle(third_command);
        break;

    default:
        break;
    }
}

static void
sbv_debug_tx_encoder(void)
{
    memset(tx_logging_buffer, 0, sizeof(tx_logging_buffer));

    sbv_control_balance_get_encoder (&(sbv_control_balance), tx_logging_buffer, SBV_DEBUG_TX_MAX_DATA_LEN);

    sbv_uart_tx_send_data(sbv_debug_interface.uart_instance, (uint8_t*)tx_logging_buffer,
                          strlen(tx_logging_buffer), SBV_DEBUG_TX_TIMEOUT_MS);
}

static void
sbv_debug_tx_imu(void)
{
    memset(tx_logging_buffer, 0, sizeof(tx_logging_buffer));

    sbv_control_balance_get_imu (&(sbv_control_balance), tx_logging_buffer, SBV_DEBUG_TX_MAX_DATA_LEN);

    sbv_uart_tx_send_data(sbv_debug_interface.uart_instance, (uint8_t*)tx_logging_buffer,
                          strlen(tx_logging_buffer), SBV_DEBUG_TX_TIMEOUT_MS);
}

static void
sbv_debug_tx_pid(void)
{
    sbv_pid_t *pid = NULL;

    memset(tx_logging_buffer, 0, sizeof(tx_logging_buffer));

    switch (sbv_tx_debug_command)
    {
    case SBV_DEBUG_TX_STEERING_PID:
        pid = &(sbv_control_balance.sbv_control_speed.steering_pid);
        break;
    case SBV_DEBUG_TX_SPEED_PID_LEFT:
        pid = &(sbv_control_balance.sbv_control_speed.motor_left.motor_pid);
        break;
    case SBV_DEBUG_TX_SPEED_PID_RIGHT:
        pid = &(sbv_control_balance.sbv_control_speed.motor_right.motor_pid);
        break;
    case SBV_DEBUG_TX_BALANCE_PID:
        pid = &(sbv_control_balance.balance_pid);
        break;
    default:
        break;
    }

    if(pid == NULL)
        return;

    sbv_control_balance_get_pid (&(sbv_control_balance), pid,
                                tx_logging_buffer, SBV_DEBUG_TX_MAX_DATA_LEN);

    sbv_uart_tx_send_data(sbv_debug_interface.uart_instance, (uint8_t*)tx_logging_buffer,
                          strlen(tx_logging_buffer), SBV_DEBUG_TX_TIMEOUT_MS);
}

void
sbv_debug_tx_logging(void)
{
    switch (sbv_tx_debug_command)
    {
    case SBV_DEBUG_TX_ENCODER:
        sbv_debug_tx_encoder();
        break;

    case SBV_DEBUG_TX_IMU:
        sbv_debug_tx_imu();
        break;

    case SBV_DEBUG_TX_STEERING_PID:
    case SBV_DEBUG_TX_SPEED_PID_LEFT:
    case SBV_DEBUG_TX_SPEED_PID_RIGHT:
    case SBV_DEBUG_TX_BALANCE_PID:
        sbv_debug_tx_pid();
        break;

    default:
        break;
    }
}