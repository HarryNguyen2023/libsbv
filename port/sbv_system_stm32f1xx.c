#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "sbv.h"
#include "sbv_system_stm32f1xx.h"

uint32_t
sbv_system_stm32f1xx_get_uid (void) {
    return HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
}

void
sbv_system_stm32f1xx_reset (void) {
    HAL_NVIC_SystemReset ();
}

void
sbv_system_stm32f1xx_application_shift_to_addr (uint32_t new_app_addr) {
    /* Disable all interrupts */
    __disable_irq();

    /* Reset the Clock */
    HAL_RCC_DeInit();
    HAL_DeInit();

    /* Clear all pending interrupts */
    NVIC_ClearPendingIRQ((IRQn_Type)0);

    /* Set new Vector Table Offset */
    SCB->VTOR = new_app_addr;

    /* Set the main stack pointer to the application slot */
    __set_MSP(*(volatile uint32_t*) new_app_addr);

    /* Disable Systick interrupt */
    SysTick->CTRL   = 0;
    SysTick->LOAD   = 0;
    SysTick->VAL    = 0;
}

void
sbv_system_stm32f1xx_delay (uint32_t delay_ms) {
    HAL_Delay (delay_ms);
}