# libsbv

`libsbv` is an embedded firmware support library for the SBV project. It provides common hardware abstractions, real-time task helpers, OTA update support, and board-specific drivers for STM32F1xx and ESP32 platforms.

## Key features

- Cross-platform support for:
  - STM32F1xx (HAL + FreeRTOS / CMSIS-RTOS)
  - ESP32 (ESP-IDF)
- OTA update framework with metadata, flash slot management, and UART/CAN transport support
- Common RTOS utilities and wrappers around FreeRTOS primitives
- Control modules for balancing, speed, and motion
- Hardware adapter modules for GPIO, UART, CAN, I2C, IMU, motor, and more
- Circular queue buffer helper for streaming packet and firmware data
- Debug and utility modules for logging and runtime state

## Repository layout

- `control/` - control algorithms and motor balancing/speed control modules
- `hw/` - hardware abstraction layer for board peripherals
- `ota/` - OTA firmware update code, packet framing, and flash update logic
- `port/` - board-specific driver ports for STM32 and ESP32
- `sbv.h` - base header with platform selection and status macros
- `sbv_config.h` - central library configuration switches
- `sbv_cqbuff.*` - circular queue buffer helper implementation
- `sbv_debug.*` - debug helper and logging support
- `sbv_pid.*` - PID controller implementation
- `sbv_rtos.h` - RTOS wrappers and helper macros
- `sbv_task.*` - SBV task entry points and thread helpers
- `sbv_wifi.*` - WiFi helper support for ESP32/related boards

## Configuration

Edit `sbv_config.h` to configure the library for your hardware and feature set.

Common options:
- `STM32F1xx` or `ESP32xx_IDF` to select target platform
- `SBV_MPU9250` to enable MPU9250 IMU support
- `SBV_DEBUG` to enable debug logging
- `SBV_HW_CRC` to use hardware CRC if available
- `SBV_OTA_CAN` or `SBV_OTA_UART` for OTA transport layer

## Usage

1. Clone or add `libsbv` as a submodule in your application repository.
2. Include `libsbv` source files in your project build.
3. Configure the target platform and OTA transport in `sbv_config.h`.
4. Use the public APIs from headers such as `sbv.h`, `sbv_ota.h`, `sbv_task.h`, and `sbv_rtos.h`.

Example:

```c
#include "sbv.h"
#include "sbv_ota.h"
#include "sbv_task.h"

int main(void)
{
    sbv_init();
    sbv_task_init();
    sbv_ota_update_init();
    sbv_rtos_start_task_scheduler();
    return 0;
}
```

## OTA update

The `ota/` module supports firmware metadata, CRC validation, and slot-based flash layout for STM32F1xx.

- `sbv_ota.h` defines firmware metadata structures and OTA slot management
- `sbv_ota_msg.c` handles OTA packet framing, command/response flows, and data transfer
- Configure transport using `SBV_OTA_UART` or `SBV_OTA_CAN`

## Building

This library is intended to be compiled as part of an embedded firmware project.

- For STM32F1xx, include `libsbv` sources in your `Makefile` or IDE project
- For ESP32, add sources to your ESP-IDF component build

## Notes

- `libsbv` is designed for embedded real-time applications and depends on FreeRTOS or CMSIS-RTOS primitives.
- The codebase currently assumes a slot-based OTA layout for STM32F1 flash memory.
- Keep `.gitmodules` and submodule references updated when using this library as a repository dependency.

## License

> Add your preferred license here
