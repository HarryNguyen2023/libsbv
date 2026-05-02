#ifndef SBV_H
#define SBV_H

#include <stdio.h>
#include "sbv_config.h"

#ifdef STM32F1xx
#include "stm32f1xx_hal.h"
#elif defined ESP32xx_IDF
#include "esp_system.h"
#endif /*STM32F1xx*/

#ifdef STM32F1xx
#define SBV_OK      HAL_OK
#define SBV_ERROR   HAL_ERROR   
#define SBV_BUSY    HAL_BUSY
#define SBV_TIMEOUT HAL_TIMEOUT

#define SBV_TRUE    1
#define SBV_FALSE   0

#elif defined ESP32xx_IDF
#define SBV_OK      ESP_OK
#define SBV_ERROR   1u

#define SBV_TRUE    TRUE
#define SBV_FALSE   FALSE

#else 
#define SBV_OK      0u
#define SBV_ERROR   1u   
#define SBV_BUSY    2u
#define SBV_TIMEOUT 3u

#define SBV_TRUE    1
#define SBV_FALSE   0

typedef unsigned char   uint8_t;
typedef char            int8_t;
typedef unsigned short  uint16_t;
typedef short           int16_t;
typedef unsigned int    uint32_t;
typedef int             int32_t;
#endif /*STM32F1xx*/

#endif /*SBV_H*/