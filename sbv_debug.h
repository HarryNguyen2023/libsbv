#ifndef SBV_DEBUG_H
#define SBV_DEBUG_H

#include "sbv.h"

/*
 * This module is used for developer debugging purpose during the development
 * of the robot system via using any console application + UART for data transmission,
 * so that the developer can set/get or even reset the value of some modules on
 * the robot system at running time, without having to rebuild the firmware and reload.
 * 
 * User guide:
 *      - There are 3 different modes for user to interract via SBV DEBUG,
 *        and for entering each mode, we must use a different CLIs
 *          + Set value:    s
 *          + Get value:    g
 *          + Reset value:  r
 *      - Then, for each different hardware modules to interract with, we also
 *        need to specify the modules name inside of the associated CLIs
 *          + Encoder value:    e
 *          + PID gain:         p
 *          + PID target:       t
 *          + IMU value:        i
 *      - Next, the SBV DEBUG allows us to specify the PID module that we want
 *        to interract with, including 3 modules
 *          + Speed control PID:    S
 *          + Steering control PID: T
 *          + Balance control PID:  B
 *      - Finally, for modules that relevant to speed control, we can specify the
 *        motor that we want to interract in the 2 motors of the Self-balanced robot
 *          + Motor left:   L
 *          + Motor right:  R
 * 
 *      - By incorporate the above characters using space as the seperation, we can
 *        create meaningful and easy to remember CLIs for debugging purpose, and the
 *        maximum number of value that can be read by the SBV DEBUG is 7 for setting
 *        the 3 PID gains of the speed control PID module, and the value associated to
 *        be set should not be longer than 4 characters.
 *        For example:
 *          + We want to set the speed target for the motor left of SBV to be 0.5m/s
 *              s t S L 0.5
 *          + We want to view the value returned by the IMU
 *              g i
 *          + We want to set the PID gain for balanced control PID (Kp:1,Ki:0.2,KD:0.1)
 *              s p B 1 0.2 0.1
 * 
 *      - Notice that the value will be output via UART to console every 100ms, to not
 *        waste the executing time of more critical tasks of the SBV robot system, and
 *        we must wire the UART correctly and set the baudrate of 115200kbps for the
 *        console, as well as enable the sbv_task_debug_console_task for transmitting
 *        and the sbv_task_uart_rx_task task for reception of commands in the sbv_task.c
 *        file when building the firmware to enable this feature. Thanks for reading.
 */

#define SBV_SET_VALUE           's'
#define SBV_GET_VALUE           'g'
#define SBV_RESET_VALUE         'r'

#define SBV_ENCODER             'e'
#define SBV_PID_GAIN            'p'
#define SBV_PID_TARGET          't'
#define SBV_IMU                 'i'

#define SBV_STEERING_PID        'T'
#define SBV_SPEED_PID           'S'
#define SBV_BALANCE_PID         'B'

#define SBV_DEBUG_MOTOR_LEFT    'L'
#define SBV_DEBUG_MOTOR_RIGHT   'R'

typedef enum sbv_debug_tx_t
{
    SBV_DEBUG_TX_NONE,
    SBV_DEBUG_TX_ENCODER,
    SBV_DEBUG_TX_IMU,
    SBV_DEBUG_TX_STEERING_PID,
    SBV_DEBUG_TX_SPEED_PID_LEFT,
    SBV_DEBUG_TX_SPEED_PID_RIGHT,
    SBV_DEBUG_TX_BALANCE_PID
} sbv_debug_tx_t;

void
sbv_debug_command_handle(char* rcv_buffer, uint16_t size);
void
sbv_debug_tx_logging(void);

#endif /*SBV_DEBUG_H*/