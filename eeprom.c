// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "eeprom.h"
#include <stdlib.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t eeprom_read(eeprom_configuration_t* configuration, uint16_t address, uint8_t* out_data, size_t length) {
    uint16_t position = 0;
    while (length - position > 0) {
        uint8_t transaction_length = length - position;
        if (transaction_length > configuration->page_size) transaction_length = configuration->page_size;
        uint8_t write_buffer[2] = {
            (uint8_t)((address + position) >> 8),
            (uint8_t)((address + position) & 0xFF),
        };
        esp_err_t res = i2c_master_transmit_receive(
            configuration->device, &write_buffer[configuration->address_16bit ? 0 : 1],
            configuration->address_16bit ? 2 : 1, &out_data[position], transaction_length, configuration->timeout_ms);
        if (res != ESP_OK) {
            return res;
        }
        position += transaction_length;
    }
    return ESP_OK;
}

esp_err_t eeprom_write(eeprom_configuration_t* configuration, uint16_t address, uint8_t* data, size_t length) {
    size_t   buffer_size = (configuration->address_16bit ? 2 : 1) +
                           (length >= configuration->page_size ? configuration->page_size : length);
    uint8_t* buffer      = malloc(buffer_size);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    uint16_t position = 0;
    while (length - position > 0) {
        uint8_t transaction_length = length - position;
        if (transaction_length > configuration->page_size) transaction_length = configuration->page_size;
        if (configuration->address_16bit) {
            buffer[0] = (address + position) >> 8;
            buffer[1] = (address + position) & 0xFF;
            memcpy(&buffer[2], &data[position], transaction_length);
        } else {
            buffer[0] = (address + position) & 0xFF;
            memcpy(&buffer[1], &data[position], transaction_length);
        }
        esp_err_t res =
            i2c_master_transmit(configuration->device, buffer,
                                transaction_length + (configuration->address_16bit ? 2 : 1), configuration->timeout_ms);
        if (res != ESP_OK) {
            free(buffer);
            return res;
        }
        position += transaction_length;
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Give the EEPROM some time to process the write command
    }
    free(buffer);
    return ESP_OK;
}
