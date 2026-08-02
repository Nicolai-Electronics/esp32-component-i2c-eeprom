// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

// Minimal stand-in for ESP-IDF's driver/i2c_master.h, used to build eeprom.c on the host for
// testing. Only declares the pieces eeprom.c actually uses; the definitions are provided by
// mock_i2c.cpp, which forwards calls to a GMock object.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct i2c_master_dev_t* i2c_master_dev_handle_t;

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t i2c_dev, const uint8_t* write_buffer, size_t write_size,
                              int xfer_timeout_ms);

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t i2c_dev, const uint8_t* write_buffer, size_t write_size,
                                      uint8_t* read_buffer, size_t read_size, int xfer_timeout_ms);

#ifdef __cplusplus
}
#endif
