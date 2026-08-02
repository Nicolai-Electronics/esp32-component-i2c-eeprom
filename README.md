# ESP-IDF component for for I2C EEPROM

An ESP-IDF component for communicating with I2C EEPROM memory chips.

## Usage

### Creating the EEPROM configuration struct

For AT24CS64 (page size of 32 bytes, 8-bit data addressing):

```
eeprom_configuration_t configuration = {
    .device = i2c_device_handle,
    .page_size = 16,
    .address_16bit = false,
    .timeout_ms = 100,
};
```

For AT24C512 (page size of 128 bytes, 16-bit data addressing):

```
eeprom_configuration_t configuration = {
    .device = i2c_device_handle,
    .page_size = 128,
    .address_16bit = true,
    .timeout_ms = 100,
};
```

### Reading from an EEPROM

```
uint8_t data[128];

esp_err_t res = eeprom_read(&configuration, 0, data, sizeof(data));
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

esp_err_t res = eeprom_write(&configuration, 0, data, sizeof(data)); // Write 128 bytes to EEPROM at data address 0
if (res == ESP_OK) {
    printf("Data written to EEPROM\r\n");
} else {
    printf("Failed to write to EEPROM (%u)\r\n", res);
}
```
