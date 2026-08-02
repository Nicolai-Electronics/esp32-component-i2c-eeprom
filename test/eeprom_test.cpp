// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include "mock_i2c.h"

extern "C" {
#include "eeprom.h"
}

using ::testing::_;
using ::testing::ElementsAreArray;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::StrictMock;

namespace {

    std::vector<uint8_t> to_vec(const uint8_t* data, size_t length) {
        return std::vector<uint8_t>(data, data + length);
    }

    class EepromTest : public ::testing::Test {
       protected:
        void SetUp() override {
            g_mock_i2c = &mock;
        }

        void TearDown() override {
            g_mock_i2c = nullptr;
        }

        StrictMock<MockI2CImpl> mock;
        i2c_master_dev_handle_t fake_device = reinterpret_cast<i2c_master_dev_handle_t>(0x1234);
    };

}  // namespace

// ---------------------------------------------------------------------------
// eeprom_read
// ---------------------------------------------------------------------------

TEST_F(EepromTest, Read_SinglePage_8BitAddress) {
    eeprom_configuration_t     config         = {fake_device, 32, false, 100};
    const std::vector<uint8_t> expected_write = {0x05};
    const std::vector<uint8_t> fake_data      = {0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t                    out[4]         = {0};

    EXPECT_CALL(mock, transmit_receive(fake_device, _, 1, _, 4, 100))
        .WillOnce([&](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t write_size, uint8_t* read_buffer,
                      size_t read_size, int) {
            EXPECT_THAT(to_vec(write_buffer, write_size), ElementsAreArray(expected_write));
            std::memcpy(read_buffer, fake_data.data(), read_size);
            return ESP_OK;
        });

    esp_err_t res = eeprom_read(&config, 0x05, out, sizeof(out));

    EXPECT_EQ(res, ESP_OK);
    EXPECT_THAT(to_vec(out, sizeof(out)), ElementsAreArray(fake_data));
}

TEST_F(EepromTest, Read_SinglePage_16BitAddress) {
    eeprom_configuration_t     config         = {fake_device, 32, true, 50};
    const std::vector<uint8_t> expected_write = {0x12, 0x34};
    const std::vector<uint8_t> fake_data      = {0x01, 0x02, 0x03};
    uint8_t                    out[3]         = {0};

    EXPECT_CALL(mock, transmit_receive(fake_device, _, 2, _, 3, 50))
        .WillOnce([&](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t write_size, uint8_t* read_buffer,
                      size_t read_size, int) {
            EXPECT_THAT(to_vec(write_buffer, write_size), ElementsAreArray(expected_write));
            std::memcpy(read_buffer, fake_data.data(), read_size);
            return ESP_OK;
        });

    esp_err_t res = eeprom_read(&config, 0x1234, out, sizeof(out));

    EXPECT_EQ(res, ESP_OK);
    EXPECT_THAT(to_vec(out, sizeof(out)), ElementsAreArray(fake_data));
}

TEST_F(EepromTest, Read_MultiPage_SplitsAcrossPageBoundaries) {
    // page_size=4 and length=10 forces transactions of 4, 4 and 2 bytes at addresses 0x00, 0x04
    // and 0x08.
    eeprom_configuration_t config  = {fake_device, 4, false, 10};
    uint8_t                out[10] = {0};

    InSequence seq;

    EXPECT_CALL(mock, transmit_receive(fake_device, _, 1, _, 4, 10))
        .WillOnce([](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t, uint8_t* read_buffer,
                     size_t read_size, int) {
            EXPECT_EQ(write_buffer[0], 0x00);
            std::memset(read_buffer, 0x11, read_size);
            return ESP_OK;
        });
    EXPECT_CALL(mock, transmit_receive(fake_device, _, 1, _, 4, 10))
        .WillOnce([](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t, uint8_t* read_buffer,
                     size_t read_size, int) {
            EXPECT_EQ(write_buffer[0], 0x04);
            std::memset(read_buffer, 0x22, read_size);
            return ESP_OK;
        });
    EXPECT_CALL(mock, transmit_receive(fake_device, _, 1, _, 2, 10))
        .WillOnce([](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t, uint8_t* read_buffer,
                     size_t read_size, int) {
            EXPECT_EQ(write_buffer[0], 0x08);
            std::memset(read_buffer, 0x33, read_size);
            return ESP_OK;
        });

    esp_err_t res = eeprom_read(&config, 0x00, out, sizeof(out));

    EXPECT_EQ(res, ESP_OK);
    const std::vector<uint8_t> expected = {0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22, 0x33, 0x33};
    EXPECT_THAT(to_vec(out, sizeof(out)), ElementsAreArray(expected));
}

TEST_F(EepromTest, Read_StopsAtFirstError) {
    // page_size=4 and length=10 would normally require three transactions; only the first
    // should ever be attempted once it fails.
    eeprom_configuration_t config  = {fake_device, 4, false, 10};
    uint8_t                out[10] = {0};

    EXPECT_CALL(mock, transmit_receive(fake_device, _, 1, _, 4, 10)).Times(1).WillOnce(Return(ESP_FAIL));

    esp_err_t res = eeprom_read(&config, 0x00, out, sizeof(out));

    EXPECT_EQ(res, ESP_FAIL);
}

TEST_F(EepromTest, Read_ZeroLength_DoesNotTransmit) {
    eeprom_configuration_t config = {fake_device, 32, false, 100};

    esp_err_t res = eeprom_read(&config, 0x00, nullptr, 0);

    EXPECT_EQ(res, ESP_OK);
}

// ---------------------------------------------------------------------------
// eeprom_write
// ---------------------------------------------------------------------------

TEST_F(EepromTest, Write_SinglePage_8BitAddress) {
    eeprom_configuration_t     config         = {fake_device, 32, false, 100};
    uint8_t                    data[4]        = {0x11, 0x22, 0x33, 0x44};
    const std::vector<uint8_t> expected_write = {0x05, 0x11, 0x22, 0x33, 0x44};

    EXPECT_CALL(mock, transmit(fake_device, _, 5, 100))
        .WillOnce([&](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t write_size, int) {
            EXPECT_THAT(to_vec(write_buffer, write_size), ElementsAreArray(expected_write));
            return ESP_OK;
        });
    EXPECT_CALL(mock, delay(10));

    esp_err_t res = eeprom_write(&config, 0x05, data, sizeof(data));

    EXPECT_EQ(res, ESP_OK);
}

TEST_F(EepromTest, Write_SinglePage_16BitAddress) {
    eeprom_configuration_t     config         = {fake_device, 32, true, 100};
    uint8_t                    data[4]        = {0xDE, 0xAD, 0xBE, 0xEF};
    const std::vector<uint8_t> expected_write = {0x12, 0x34, 0xDE, 0xAD, 0xBE, 0xEF};

    EXPECT_CALL(mock, transmit(fake_device, _, 6, 100))
        .WillOnce([&](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t write_size, int) {
            EXPECT_THAT(to_vec(write_buffer, write_size), ElementsAreArray(expected_write));
            return ESP_OK;
        });
    EXPECT_CALL(mock, delay(10));

    esp_err_t res = eeprom_write(&config, 0x1234, data, sizeof(data));

    EXPECT_EQ(res, ESP_OK);
}

TEST_F(EepromTest, Write_MultiPage_SplitsAndIncrementsAddress) {
    // page_size=4 and length=6 forces transactions of 4 and 2 bytes at addresses 0x00 and 0x04.
    // Regression test: a prior operator-precedence bug in buffer_size's calculation caused the
    // first (full-page) transaction here to overflow the malloc'd buffer.
    eeprom_configuration_t config  = {fake_device, 4, false, 10};
    uint8_t                data[6] = {0, 1, 2, 3, 4, 5};

    InSequence seq;

    EXPECT_CALL(mock, transmit(fake_device, _, 5, 10))
        .WillOnce([](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t write_size, int) {
            const std::vector<uint8_t> expected = {0x00, 0, 1, 2, 3};
            EXPECT_THAT(to_vec(write_buffer, write_size), ElementsAreArray(expected));
            return ESP_OK;
        });
    EXPECT_CALL(mock, delay(10));
    EXPECT_CALL(mock, transmit(fake_device, _, 3, 10))
        .WillOnce([](i2c_master_dev_handle_t, const uint8_t* write_buffer, size_t write_size, int) {
            const std::vector<uint8_t> expected = {0x04, 4, 5};
            EXPECT_THAT(to_vec(write_buffer, write_size), ElementsAreArray(expected));
            return ESP_OK;
        });
    EXPECT_CALL(mock, delay(10));

    esp_err_t res = eeprom_write(&config, 0x00, data, sizeof(data));

    EXPECT_EQ(res, ESP_OK);
}

TEST_F(EepromTest, Write_StopsAtFirstErrorWithoutDelay) {
    eeprom_configuration_t config  = {fake_device, 32, false, 100};
    uint8_t                data[4] = {0, 1, 2, 3};

    EXPECT_CALL(mock, transmit(fake_device, _, 5, 100)).Times(1).WillOnce(Return(ESP_FAIL));
    // No EXPECT_CALL for delay(): the mock is strict, so a call here would fail the test.

    esp_err_t res = eeprom_write(&config, 0x00, data, sizeof(data));

    EXPECT_EQ(res, ESP_FAIL);
}

TEST_F(EepromTest, Write_ZeroLength_DoesNotTransmit) {
    eeprom_configuration_t config  = {fake_device, 32, false, 100};
    uint8_t                data[1] = {0xFF};

    esp_err_t res = eeprom_write(&config, 0x00, data, 0);

    EXPECT_EQ(res, ESP_OK);
}
