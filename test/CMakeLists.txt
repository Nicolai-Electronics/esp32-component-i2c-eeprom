# SPDX-FileCopyrightText: 2026 Nicolai Electronics
# SPDX-License-Identifier: MIT
#
# Host-side unit tests for eeprom.c. This is a standalone CMake project (not an ESP-IDF
# component build): eeprom.c is compiled against fake ESP-IDF/FreeRTOS headers in mocks/ so the
# I2C transaction logic can be tested with GoogleTest/GoogleMock without real hardware.

cmake_minimum_required(VERSION 3.16)
project(eeprom_tests LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

enable_testing()
find_package(GTest REQUIRED)

add_executable(eeprom_tests
  eeprom_test.cpp
  mock_i2c.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../eeprom.c
)

target_include_directories(eeprom_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/mocks
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(eeprom_tests PRIVATE
  -Wall -Wextra
  -fsanitize=address,undefined
  -fno-omit-frame-pointer
)
target_link_options(eeprom_tests PRIVATE
  -fsanitize=address,undefined
)

target_link_libraries(eeprom_tests PRIVATE
  GTest::gtest
  GTest::gmock
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(eeprom_tests)
