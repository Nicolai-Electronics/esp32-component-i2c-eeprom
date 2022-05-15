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

esp_err_t _eeprom_read(int bus, uint8_t addr, uint16_t address, uint8_t* data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t        res = i2c_master_start(cmd);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, address >> 8, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, address & 0xFF, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_start(cmd);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, (addr << 1) | READ_BIT, ACK_CHECK_EN);
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

    res = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

esp_err_t _eeprom_write(int bus, uint8_t addr, uint16_t address, uint8_t* data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t        res = i2c_master_start(cmd);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, address >> 8, ACK_CHECK_EN);
    if (res != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return res;
    }
    res = i2c_master_write_byte(cmd, address & 0xFF, ACK_CHECK_EN);
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

    res = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

esp_err_t eeprom_read(EEPROM* device, uint16_t address, uint8_t* data, size_t length) {
    uint16_t position = 0;
    while (length - position > 0) {
        uint8_t transaction_length = length - position;
        if (transaction_length > 64) transaction_length = 64;
        esp_err_t res = _eeprom_read(device->i2c_bus, device->i2c_address, address + position, &data[position], transaction_length);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "EEPROM read failed (%d)", res);
            return res;
        }
        position += transaction_length;
    }
    return ESP_OK;
}

esp_err_t eeprom_write(EEPROM* device, uint16_t address, uint8_t* data, size_t length) {
    uint16_t position = 0;
    while (length - position > 0) {
        uint8_t transaction_length = length - position;
        if (transaction_length > 64) transaction_length = 64;
        esp_err_t res = _eeprom_write(device->i2c_bus, device->i2c_address, address + position, &data[position], transaction_length);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "EEPROM write failed (%d)", res);
            return res;
        }
        position += transaction_length;
    }
    return ESP_OK;
}
