// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    i2c_master_dev_handle_t device;         // I2C device handle
    uint8_t                 page_size;      // EEPROM page size in bytes
    bool                    address_16bit;  // true if the EEPROM uses 16-bit memory addresses
    int                     timeout_ms;     // I2C transfer timeout in milliseconds
} eeprom_configuration_t;

/// @brief Reads from the EEPROM using the provided configuration struct
/// @return ESP-IDF error code
esp_err_t eeprom_read(eeprom_configuration_t* configuration, uint16_t address, uint8_t* out_data, size_t length);

/// @brief Writes to the EEPROM using the provided configuration struct
/// @return ESP-IDF error code
esp_err_t eeprom_write(eeprom_configuration_t* configuration, uint16_t address, uint8_t* data, size_t length);

#ifdef __cplusplus
}
#endif
