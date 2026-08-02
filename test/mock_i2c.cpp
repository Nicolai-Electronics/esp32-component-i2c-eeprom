// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

// C-linkage definitions of the fake ESP-IDF/FreeRTOS entry points declared in
// test/mocks/driver/i2c_master.h and test/mocks/freertos/FreeRTOS.h. These forward every call
// into the currently active GMock object so tests can set expectations and inspect the exact
// bytes eeprom.c sends over I2C.
#include "mock_i2c.h"

MockI2C* g_mock_i2c = nullptr;

extern "C" {

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t i2c_dev, const uint8_t* write_buffer, size_t write_size,
                              int xfer_timeout_ms) {
    return g_mock_i2c->transmit(i2c_dev, write_buffer, write_size, xfer_timeout_ms);
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t i2c_dev, const uint8_t* write_buffer, size_t write_size,
                                      uint8_t* read_buffer, size_t read_size, int xfer_timeout_ms) {
    return g_mock_i2c->transmit_receive(i2c_dev, write_buffer, write_size, read_buffer, read_size, xfer_timeout_ms);
}

void vTaskDelay(uint32_t ticks_to_delay) {
    g_mock_i2c->delay(ticks_to_delay);
}

}  // extern "C"
