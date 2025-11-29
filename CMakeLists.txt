idf_component_register(
  SRCS
    "eeprom.c"
  INCLUDE_DIRS
    "include"
  PRIV_REQUIRES
    "esp_driver_i2c"
)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
