// SPDX-FileCopyrightText: 2025 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    i2c_master_bus_handle_t i2c_bus;               // I2C bus handle
    SemaphoreHandle_t       i2c_semaphore;         // I2C bus semaphore
    uint16_t                i2c_address;           // I2C address of the EEPROM device (7-bit)
    uint8_t                 eeprom_page_size;      // EEPROM page size in bytes
    bool                    eeprom_address_16bit;  // true if the EEPROM uses 16-bit memory addresses
    i2c_master_dev_handle_t i2c_device;            // I2C device handle
    int                     timeout_ms;            // I2C transfer timeout in milliseconds
} eeprom_handle_t;

esp_err_t eeprom_init(eeprom_handle_t* handle, i2c_master_bus_handle_t i2c_bus, SemaphoreHandle_t i2c_semaphore,
                      uint16_t i2c_address, uint8_t eeprom_page_size, bool eeprom_address_16bit);
esp_err_t eeprom_read(eeprom_handle_t* handle, uint16_t address, uint8_t* out_data, size_t length);
esp_err_t eeprom_write(eeprom_handle_t* handle, uint16_t address, uint8_t* data, size_t length);
