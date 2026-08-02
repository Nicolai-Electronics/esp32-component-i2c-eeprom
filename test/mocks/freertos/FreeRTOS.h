// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

// Minimal stand-in for FreeRTOS's FreeRTOS.h, used to build eeprom.c on the host for testing.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define portTICK_PERIOD_MS 1

void vTaskDelay(uint32_t ticks_to_delay);

#ifdef __cplusplus
}
#endif
