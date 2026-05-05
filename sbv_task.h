#ifndef SBV_TASK_H
#define SBV_TASK_H

void
sbv_init(void);
void
sbv_task_init(void);
void
sbv_task_balance_control(void *param);
// void
// sbv_task_debug_console_task(void *param);
void
sbv_task_uart_rx_task(void *param);

#endif  /*SBV_TASK_H*/