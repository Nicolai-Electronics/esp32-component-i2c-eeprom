// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

// GMock interface used by mock_i2c.cpp to stand in for ESP-IDF's I2C master driver, so that
// eeprom.c's transaction logic can be verified without real hardware or ESP-IDF.
#pragma once

#include <gmock/gmock.h>

extern "C" {
#include "driver/i2c_master.h"
}

class MockI2C {
   public:
    virtual ~MockI2C() = default;

    virtual esp_err_t transmit(i2c_master_dev_handle_t dev, const uint8_t* write_buffer, size_t write_size,
                               int timeout_ms) = 0;

    virtual esp_err_t transmit_receive(i2c_master_dev_handle_t dev, const uint8_t* write_buffer, size_t write_size,
                                       uint8_t* read_buffer, size_t read_size, int timeout_ms) = 0;

    virtual void delay(uint32_t ticks) = 0;
};

class MockI2CImpl : public MockI2C {
   public:
    MOCK_METHOD(esp_err_t, transmit, (i2c_master_dev_handle_t, const uint8_t*, size_t, int), (override));
    MOCK_METHOD(esp_err_t, transmit_receive, (i2c_master_dev_handle_t, const uint8_t*, size_t, uint8_t*, size_t, int),
                (override));
    MOCK_METHOD(void, delay, (uint32_t), (override));
};

// Set by each test fixture; consumed by the C-linkage shims in mock_i2c.cpp.
extern MockI2C* g_mock_i2c;
