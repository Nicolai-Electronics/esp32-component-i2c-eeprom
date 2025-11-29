// SPDX-FileCopyrightText: 2025 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "eeprom.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char* TAG = "EEPROM";

static void claim_i2c_bus(eeprom_handle_t* handle) {
    // Claim I2C bus
    if (handle->i2c_semaphore != NULL) {
        xSemaphoreTake(handle->i2c_semaphore, portMAX_DELAY);
    } else {
        ESP_LOGW(TAG, "No concurrency semaphore");
    }
}

static void release_i2c_bus(eeprom_handle_t* handle) {
    // Release I2C bus
    if (handle->i2c_semaphore != NULL) {
        xSemaphoreGive(handle->i2c_semaphore);
    }
}

static esp_err_t ts_i2c_master_transmit_receive(eeprom_handle_t* handle, const uint8_t* write_buffer, size_t write_size,
                                                uint8_t* read_buffer, size_t read_size) {
    claim_i2c_bus(handle);
    esp_err_t res = i2c_master_transmit_receive(handle->i2c_device, write_buffer, write_size, read_buffer, read_size,
                                                handle->timeout_ms);
    release_i2c_bus(handle);
    return res;
}

static esp_err_t ts_i2c_master_transmit(eeprom_handle_t* handle, const uint8_t* write_buffer, size_t write_size) {
    claim_i2c_bus(handle);
    esp_err_t res = i2c_master_transmit(handle->i2c_device, write_buffer, write_size, handle->timeout_ms);
    release_i2c_bus(handle);
    return res;
}

esp_err_t eeprom_init(eeprom_handle_t* handle, i2c_master_bus_handle_t i2c_bus, SemaphoreHandle_t i2c_semaphore,
                      uint16_t i2c_address, uint8_t eeprom_page_size, bool eeprom_address_16bit) {
    if (handle == NULL || i2c_bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->i2c_bus              = i2c_bus;
    handle->i2c_semaphore        = i2c_semaphore;
    handle->i2c_address          = i2c_address;
    handle->eeprom_page_size     = eeprom_page_size;
    handle->eeprom_address_16bit = eeprom_address_16bit;
    handle->timeout_ms           = 100;

    claim_i2c_bus(handle);
    esp_err_t res = i2c_master_probe(handle->i2c_bus, handle->i2c_address, handle->timeout_ms);
    release_i2c_bus(handle);

    if (res != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = handle->i2c_address,
        .scl_speed_hz    = 400000,
    };

    res = i2c_master_bus_add_device(handle->i2c_bus, &dev_cfg, &handle->i2c_device);

    return res;
}
esp_err_t eeprom_read(eeprom_handle_t* handle, uint16_t address, uint8_t* out_data, size_t length) {
    uint16_t position = 0;
    while (length - position > 0) {
        uint8_t transaction_length = length - position;
        if (transaction_length > handle->eeprom_page_size) transaction_length = handle->eeprom_page_size;
        uint8_t address_buffer[2] = {
            (uint8_t)((address + position) >> 8),
            (uint8_t)((address + position) & 0xFF),
        };
        esp_err_t res = ts_i2c_master_transmit_receive(handle, &address_buffer[handle->eeprom_address_16bit ? 0 : 1],
                                                       handle->eeprom_address_16bit ? 2 : 1, &out_data[position],
                                                       transaction_length);
        if (res != ESP_OK) {
            return res;
        }
        position += transaction_length;
    }
    return ESP_OK;
}

esp_err_t eeprom_write(eeprom_handle_t* handle, uint16_t address, uint8_t* data, size_t length) {
    size_t   buffer_size = (handle->eeprom_address_16bit ? 2 : 1) + (length >= handle->eeprom_page_size)
                               ? handle->eeprom_page_size
                               : length;
    uint8_t* buffer      = malloc(buffer_size);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    uint16_t position = 0;
    while (length - position > 0) {
        uint8_t transaction_length = length - position;
        if (transaction_length > handle->eeprom_page_size) transaction_length = handle->eeprom_page_size;
        if (handle->eeprom_address_16bit) {
            buffer[0] = (address + position) >> 8;
            buffer[1] = (address + position) & 0xFF;
            memcpy(&buffer[2], &data[position], transaction_length);
        } else {
            buffer[0] = (address + position) & 0xFF;
            memcpy(&buffer[1], &data[position], transaction_length);
        }
        printf("I2C transaction contents: ");
        for (size_t i = 0; i < transaction_length + (handle->eeprom_address_16bit ? 2 : 1); i++) {
            printf("%02X ", buffer[i]);
        }
        printf("\n");
        esp_err_t res =
            ts_i2c_master_transmit(handle, buffer, transaction_length + (handle->eeprom_address_16bit ? 2 : 1));
        if (res != ESP_OK) {
            return res;
        }
        position += transaction_length;
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Give the EEPROM some time to process the write command
    }
    return ESP_OK;
}
