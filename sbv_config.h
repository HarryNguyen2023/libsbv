#ifndef SBV_CONFIG_H
#define SBV_CONFIG_H

/*Define the suitable type of hardware use for the whole sbv library*/
#define STM32F1xx
// #define ESP32xx_IDF

/*Define the IMU type*/
#define SBV_MPU9250

/*Define wether prin debug log*/
#undef SBV_DEBUG

/*Define whether use HW CRC module*/
#undef SBV_HW_CRC

/*Define the data link layer for OTA*/
// #define SBV_OTA_CAN
#define SBV_OTA_UART

#endif  /*SBV_CONFIG_H*/