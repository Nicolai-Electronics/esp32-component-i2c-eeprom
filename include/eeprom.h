#pragma once

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int i2c_bus;
    int i2c_address;
    bool address_16bit;
    uint8_t page_size;
} EEPROM;

esp_err_t eeprom_init(EEPROM* device);
esp_err_t eeprom_read(EEPROM* device, uint16_t address, uint8_t* data, size_t length);
esp_err_t eeprom_write(EEPROM* device, uint16_t address, uint8_t* data, size_t length);
