# ESP32 component: EEPROM

An ESP-IDF component for communicating with the EEPROM memory chips.

## Usage

### Reading from an EEPROM

```
esp_err_t eeprom_read(EEPROM* device, uint16_t address, uint8_t* data, size_t length)
```

### Writing to an EEPROM

```
esp_err_t eeprom_write(EEPROM* device, uint16_t address, uint8_t* data, size_t length)
```
