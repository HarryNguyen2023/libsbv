#ifndef SBV_RTOS_H
#define SBV_RTOS_H

#include "sbv.h"

#ifdef STM32F1xx
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#include "list.h"
#include "message_buffer.h"
#include "stream_buffer.h"
#include "queue.h"
#include "semphr.h"

#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/event_groups.h"
#include "freertos/list.h"
#include "freertos/message_buffer.h"
#include "freertos/stream_buffer.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#endif

#define sbv_rtos_ms_to_tick     pdMS_TO_TICKS
#define SBV_RTOS_MAX_DELAY      sbv_rtos_ms_to_tick(1000)
#define SBV_RTOS_FALSE          pdFALSE
#define SBV_RTOS_TRUE           pdTRUE


typedef SemaphoreHandle_t       sbv_rtos_mutex_t;
typedef BaseType_t              sbv_rtos_base_type_t;
typedef TickType_t              sbv_rtos_tick_type_t;
typedef TaskHandle_t            sbv_rtos_task_handle_t;
typedef TimeOut_t               sbv_rtos_timeout_t;
typedef EventGroupHandle_t      sbv_event_group_handle_t;
typedef EventBits_t             sbv_rtos_event_bits_t;

#define sbv_rtos_mutex_create(M)  \
        M = xSemaphoreCreateMutex()

#define sbv_rtos_mutex_lock(M)  \
        xSemaphoreTake(M, SBV_RTOS_MAX_DELAY)

#define sbv_rtos_mutex_unlock(M)  \
        xSemaphoreGive(M)

#define sbv_rtos_malloc(S)  \
        pvPortMalloc(S)

#define sbv_rtos_free(P)  \
        vPortFree(P)

#define sbv_rtos_notify_give_fromISR(T, W)  \
        vTaskNotifyGiveFromISR(T, W)

#define sbv_rtos_notify_take(S, T)  \
        ulTaskNotifyTake(S, T)

#define sbv_rtos_get_current_task_handle()  \
        xTaskGetCurrentTaskHandle()

#define sbv_rtos_set_timeout_state(T)  \
        vTaskSetTimeOutState(T)

#define sbv_rtos_check_for_timeout(T, W)  \
        xTaskCheckForTimeOut(T, W)

#define sbv_rtos_port_yield_fromISR(H)  \
        portYIELD_FROM_ISR(H)

#define sbv_rtos_task_create(Function, Name, Stack_depth, Param, Priority, TaskHandle)  \
        xTaskCreate(Function, Name, Stack_depth, Param, Priority, TaskHandle)

#define sbv_rtos_task_delete(Task_handle)  \
        vTaskDelete(Task_handle)

#define sbv_rtos_task_delay(T)  \
        vTaskDelay(T)

#define sbv_rtos_start_task_scheduler()  \
        vTaskStartScheduler()

#define sbv_rtos_event_group_create()  \
        xEventGroupCreate()

#define sbv_rtos_event_group_set_bits(G, B)  \
        xEventGroupSetBits(G, B)

#define sbv_rtos_event_group_wait_bits(G, B, Clear, All, T)  \
        xEventGroupWaitBits(G, B, Clear, All, T)

#define sbv_rtos_event_group_delete(G)  \
        vEventGroupDelete(G)

#endif  /*SBV_RTOS_H*/