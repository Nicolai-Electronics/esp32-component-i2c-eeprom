idf_component_register(
  SRCS
    "eeprom.c"
  INCLUDE_DIRS
    "include"
  PRIV_REQUIRES
    "esp_driver_i2c"
)
