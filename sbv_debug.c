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

#define SBV_DEBUG_TX_TIMEOUT_MS     (100)

char rcv_command[7][5];
sbv_debug_tx_t sbv_tx_debug_command = SBV_DEBUG_TX_NONE;
char tx_logging_buffer[100];

extern sbv_control_balance_t sbv_control_balance;
extern sbv_uart_handle_t     huart1;

static void
sbv_debug_set_command_handle(void);
static void
sbv_debug_get_command_handle(void);
static void
sbv_debug_reset_command_handle(void);

static void
sbv_debug_break_command(char* rcv_buffer, uint16_t size)
{
    uint8_t i = 0, max_idx = sizeof(rcv_command) / sizeof(rcv_command[0]);
    char *token;
    char *saveptr;

    if(!rcv_buffer || size == 0)
        return;

    memset(rcv_command, 0, sizeof(rcv_command));

    token = strtok_r(rcv_buffer, " ", &saveptr);

    while (token != NULL) 
    {
        strcpy(rcv_command[i++], token);
        token = strtok_r(NULL, " ", &saveptr);

        /* Avoid buffer overflow */
        if(i == max_idx)
            break;
    }
}

void
sbv_debug_command_handle(char* rcv_buffer, uint16_t size)
{
    uint8_t first_command;

    if(!rcv_buffer || size == 0)
        return;

    sbv_debug_break_command(rcv_buffer, size);

    if((strlen(rcv_command[0]) != 1) || (strlen(rcv_command[1]) != 1))
    {
        sbv_tx_debug_command = SBV_DEBUG_TX_NONE;
        return;
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

        sbv_pid_set_gain(&(sbv_control_balance.sbv_control_speed.steering_pid),
                        pid_gains[0], pid_gains[1], pid_gains[2]);
        break;

    case SBV_BALANCE_PID:
        /* Get 3 of the gains */
        for(i = 0; i < 3; ++i)
            pid_gains[i] = strtof(rcv_command[3 + i], NULL);

        sbv_pid_set_gain(&(sbv_control_balance.balance_pid),
                        pid_gains[0], pid_gains[1], pid_gains[2]);
        break;

    case SBV_SPEED_PID:
        /* Get the name of motor */
        motor_name = rcv_command[3][0];

        /* Get 3 of the gains */
        for(i = 0; i < 3; ++i)
            pid_gains[i] = strtof(rcv_command[4 + i], NULL);

        if(motor_name == SBV_DEBUG_MOTOR_LEFT)
            sbv_pid_set_gain(&(sbv_control_balance.sbv_control_speed.motor_left.motor_pid),
                            pid_gains[0], pid_gains[1], pid_gains[2]);
        else if(motor_name == SBV_DEBUG_MOTOR_RIGHT)
            sbv_pid_set_gain(&(sbv_control_balance.sbv_control_speed.motor_right.motor_pid),
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

        sbv_pid_set_target(&(sbv_control_balance.sbv_control_speed.steering_pid), pid_target);
        break;

    case SBV_BALANCE_PID:
        pid_target = strtof(rcv_command[3], NULL);

        sbv_pid_set_target(&(sbv_control_balance.balance_pid), pid_target);
        break;

    case SBV_SPEED_PID:
        /* Get the name of motor */
        motor_name = rcv_command[3][0];

        pid_target = strtof(rcv_command[4], NULL);

        if(motor_name == SBV_DEBUG_MOTOR_LEFT)
            sbv_pid_set_target(&(sbv_control_balance.sbv_control_speed.motor_left.motor_pid), pid_target);
        else if(motor_name == SBV_DEBUG_MOTOR_RIGHT)
            sbv_pid_set_target(&(sbv_control_balance.sbv_control_speed.motor_right.motor_pid), pid_target);
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
        sbv_pid_reset(&(sbv_control_balance.sbv_control_speed.steering_pid));
        break;

    case SBV_BALANCE_PID:
        sbv_pid_reset(&(sbv_control_balance.balance_pid));
        break;

    case SBV_SPEED_PID:
        motor_name = rcv_command[3][0];
        if(motor_name == SBV_DEBUG_MOTOR_LEFT)
            sbv_pid_reset(&(sbv_control_balance.sbv_control_speed.motor_left.motor_pid));
        else if(motor_name == SBV_DEBUG_MOTOR_RIGHT)
            sbv_pid_reset(&(sbv_control_balance.sbv_control_speed.motor_right.motor_pid));
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
        sbv_motor_encoder_reset((sbv_control_balance.sbv_control_speed.motor_left.motor));
        sbv_motor_encoder_reset((sbv_control_balance.sbv_control_speed.motor_right.motor));
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

    sprintf(tx_logging_buffer, "\r\n%u,%u",
            sbv_motor_read_encoder((sbv_control_balance.sbv_control_speed.motor_left.motor)),
            sbv_motor_read_encoder((sbv_control_balance.sbv_control_speed.motor_right.motor)));

    sbv_uart_tx_send_data((uint8_t*)tx_logging_buffer, strlen(tx_logging_buffer), SBV_DEBUG_TX_TIMEOUT_MS);
}

static void
sbv_debug_tx_imu(void)
{
    memset(tx_logging_buffer, 0, sizeof(tx_logging_buffer));

    sprintf(tx_logging_buffer, "\r\n%.4f,%.4f,%.4f,%.4f",
            sbv_control_balance.imu->theta.est_sensor,
            sbv_control_balance.imu->theta.est_post,
            sbv_control_balance.imu->phi.est_sensor,
            sbv_control_balance.imu->phi.est_post);

    sbv_uart_tx_send_data((uint8_t*)tx_logging_buffer, strlen(tx_logging_buffer), SBV_DEBUG_TX_TIMEOUT_MS);
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

    sprintf(tx_logging_buffer, "\r\n%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
            pid->target, pid->feedback, pid->output,
            pid->Kp, pid->Ki, pid->Kd);

    sbv_uart_tx_send_data((uint8_t*)tx_logging_buffer, strlen(tx_logging_buffer), SBV_DEBUG_TX_TIMEOUT_MS);
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