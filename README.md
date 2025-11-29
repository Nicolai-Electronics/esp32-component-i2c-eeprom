# ESP-IDF component for for I2C EEPROM

An ESP-IDF component for communicating with I2C EEPROM memory chips.

## Usage

### Initializing the EEPROM handle struct

For AT24CS64 (page size of 32 bytes, 8-bit data addressing):

```
eeprom_handle_t handle = {0};
esp_err_t res = eeprom_init(&handle, i2c_bus, i2c_semaphore, 0x50, 16, false);

if (res == ESP_ERR_NOT_FOUND) {
    printf("EEPROM not found on I2C bus\r\n");
} else if (res != ESP_OK) {
    printf("EEPROM initialization error (%u)\r\n", res);
} else {
    printf("EEPROM initialized\r\n");
}
```

For AT24C512 (page size of 128 bytes, 16-bit data addressing):

```
eeprom_handle_t handle = {0};
esp_err_t res = eeprom_init(&handle, i2c_bus, i2c_semaphore, 0x50, 128, true);

if (res == ESP_ERR_NOT_FOUND) {
    printf("EEPROM not found on I2C bus\r\n");
} else if (res != ESP_OK) {
    printf("EEPROM initialization error (%u)\r\n", res);
} else {
    printf("EEPROM initialized\r\n");
}
```

### Reading from an EEPROM

```

uint8_t data[128];

esp_err_t res = eeprom_read(&handle, 0, data, sizeof(data)); // Read 128 bytes from EEPROM at data address 0
if (res == ESP_OK) {
    printf("Read data from EEPROM: ");
    for (size_t i = 0; i < sizeof(data); i++) {
        printf("%02X ");
    }
    printf("\r\n");
} else {
    printf("Failed to read from EEPROM (%u)\r\n", res);
}
```

### Writing to an EEPROM

```
uint8_t data[128];
for (size_t i = 0; i < sizeof(data); i++) data[i] = i; // Fill buffer with test pattern

esp_err_t res = eeprom_write(&handle, 0, data, sizeof(data)); // Write 128 bytes to EEPROM at data address 0
if (res == ESP_OK) {
    printf("Data written to EEPROM\r\n");
} else {
    printf("Failed to write to EEPROM (%u)\r\n", res);
}
```
