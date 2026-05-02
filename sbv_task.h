#ifndef SBV_TASK_H
#define SBV_TASK_H

void
sbv_init(void);
void
sbv_task_init(void);
void
sbv_task_speed_control(void *param);
void
sbv_task_speed_twist_control(void *param);
void
sbv_task_balance_control(void *param);
void
sbv_task_debug_console_task(void *param);
void
sbv_task_uart_rx_task(void *param);
void
sbv_task_can_tx_debug_task(void *param);
void
sbv_task_can_rx_debug_task(void *param);
void
sbv_task_can_rx_handle(void *param);

#endif  /*SBV_TASK_H*/