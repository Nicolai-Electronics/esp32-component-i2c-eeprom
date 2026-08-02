// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

// Minimal stand-in for ESP-IDF's esp_err.h, used to build eeprom.c on the host for testing.
//
// Real ESP-IDF headers transitively pull in <stdlib.h>/<string.h> (eeprom.c relies on this for
// malloc/memcpy/free without including them directly); this header is always reachable from
// eeprom.c (directly, and via eeprom.h's `#include <esp_err.h>`), so the shim lives here.
#pragma once

#include <stdlib.h>
#include <string.h>

typedef int esp_err_t;

#define ESP_OK                0
#define ESP_FAIL              -1
#define ESP_ERR_NO_MEM        0x101
#define ESP_ERR_INVALID_ARG   0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE  0x104
#define ESP_ERR_NOT_FOUND     0x105
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT       0x107
