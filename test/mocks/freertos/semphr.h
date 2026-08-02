// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

// Minimal stand-in for FreeRTOS's semphr.h, used to build eeprom.c on the host for testing.
#pragma once

typedef void* SemaphoreHandle_t;
