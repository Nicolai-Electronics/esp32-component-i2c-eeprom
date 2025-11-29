/**
 * Copyright (c) 2022 Nicolai Electronics
 *
 * SPDX-License-Identifier: MIT
 */

#include "eeprom.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <sdkconfig.h>

#define WRITE_BIT     I2C_MASTER_WRITE /* I2C master write */
#define READ_BIT      I2C_MASTER_READ  /* I2C master read */
#define ACK_CHECK_EN  0x1              /* I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0              /* I2C master will not check ack from slave */
#define ACK_VAL       0x0              /* I2C ack value */
#define NACK_VAL      0x1              /* I2C nack value */

static const char* TAG = "EEPROM";

esp_err_t eeprom_init(EEPROM* device) { return ESP_OK; }

esp_err_t _eeprom_read(int bus, uint8_t i2c_address, uint16_t memory_address, uint8_t* data, size_t length, bool address_16bit) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t        res = i2c_master_start(cmd);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, (i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    if (address_16bit) {
        res = i2c_master_write_byte(cmd, memory_address >> 8, ACK_CHECK_EN);
        if (res != ESP_OK) {
            i2c_cmd_link_delete(cmd);
            return res;
        }
    }
    res = i2c_master_write_byte(cmd, memory_address & 0xFF, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_start(cmd);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, (i2c_address << 1) | READ_BIT, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    if (length > 1) {
        res = i2c_master_read(cmd, data, length - 1, ACK_VAL);
        if (res != ESP_OK) {
            i2c_cmd_link_delete(cmd);
            return res;
        }
    }
    res = i2c_master_read_byte(cmd, &data[length - 1], NACK_VAL);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_stop(cmd);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }

    res = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

esp_err_t _eeprom_write(int bus, uint8_t i2c_address, uint16_t memory_address, uint8_t* data, size_t length, bool address_16bit) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t        res = i2c_master_start(cmd);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, (i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    if (address_16bit) {
        res = i2c_master_write_byte(cmd, memory_address >> 8, ACK_CHECK_EN);
        if (res != ESP_OK) {
            i2c_cmd_link_delete(cmd);
            return res;
        }
    }
    res = i2c_master_write_byte(cmd, memory_address & 0xFF, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    for (size_t i = 0; i < length; i++) {
        res = i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
        if (res != ESP_OK) {
            i2c_cmd_link_delete(cmd);
            return res;
        }
    }
    res = i2c_master_stop(cmd);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }

    res = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

esp_err_t eeprom_read(EEPROM* device, uint16_t address, uint8_t* data, size_t length) {
    uint16_t position = 0;
    while (length - position > 0) {
        uint8_t transaction_length = length - position;
        if (transaction_length > device->page_size) transaction_length = device->page_size;
        esp_err_t res = _eeprom_read(device->i2c_bus, device->i2c_address, address + position, &data[position], transaction_length, device->address_16bit);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "EEPROM read failed (%d)", res);
            return res;
        }
        position += transaction_length;
    }
    return ESP_OK;
}

esp_err_t eeprom_write(EEPROM* device, uint16_t address, uint8_t* data, size_t length) {
    printf("EEPROM write %u\n", length);
    uint16_t position = 0;
    while (length - position > 0) {
        uint8_t transaction_length = length - position;
        if (transaction_length > device->page_size) transaction_length = device->page_size;
        printf("Writing at %u: ", address + position);
        for (int i = position; i < position + transaction_length; i++) {
            printf("%02x ", data[i]);
        }
        printf("\n");
        esp_err_t res = _eeprom_write(device->i2c_bus, device->i2c_address, address + position, &data[position], transaction_length, device->address_16bit);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "EEPROM write failed (%d)", res);
            return res;
        }
        position += transaction_length;
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Give the EEPROM some time to process the write command
    }
    return ESP_OK;
}
