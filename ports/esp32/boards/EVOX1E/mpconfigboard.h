#ifndef MICROPY_HW_BOARD_NAME
// Can be set by mpconfigboard.cmake.
#define MICROPY_HW_BOARD_NAME               "EVO-X1E"
#endif
#define MICROPY_HW_MCU_NAME                 "ESP32S3"

// Enable UART REPL for modules that have an external USB-UART and don't use native USB.
#define MICROPY_HW_ENABLE_UART_REPL         (1)

#define MICROPY_HW_I2C0_SCL                 (1)
#define MICROPY_HW_I2C0_SDA                 (2)

#define EVO_DEFAULT_NAME "Evo_X1E"
#define MICROPY_HW_USB_MANUFACTURER_STRING "TribalStudioz"
#define MICROPY_HW_USB_PRODUCT_FS_STRING EVO_DEFAULT_NAME
#define EVO_DEVICE_TYPE "EVO-X1E"
#define EVO_CONTROLLER_TYPE "EVO-X1E"
#define EVO_DEFAULT_BLE_DOWNLOAD_ENABLED (false)
#define EVO_DEFAULT_BLE_DOWNLOAD_ON_BOOT (false)
