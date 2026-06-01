#ifndef MICROPY_HW_BOARD_NAME
// Can be set by mpconfigboard.cmake.
#define MICROPY_HW_BOARD_NAME               "EVO-X1P"
#endif
#define MICROPY_HW_MCU_NAME                 "ESP32S3"

// Enable UART REPL for modules that have an external USB-UART and don't use native USB.
#define MICROPY_HW_ENABLE_UART_REPL         (1)

#define MICROPY_HW_I2C0_SCL                 (1)
#define MICROPY_HW_I2C0_SDA                 (2)

#define EVO_DEFAULT_NAME "Evo_X1P"
#define EVO_DEVICE_TYPE "EVO-X1P"
#define EVO_CONTROLLER_TYPE "EVO-X1P"
#define EVO_MULTIPLE_PROGRAM_FILESYSTEM (true)
#define EVO_DEFAULT_BLE_DOWNLOAD_ENABLED (true)
#define EVO_DEFAULT_BLE_DOWNLOAD_ON_BOOT (false)
