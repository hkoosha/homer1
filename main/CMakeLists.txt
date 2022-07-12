# set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=clang-analyzer-core.uninitialized.Branch)

idf_component_register(
        SRCS
            "homer1.cpp"
            "homer1.hpp"
            "homer_helper.c"
            "Adafruit_CCS811.cpp"
            "Adafruit_CCS811.h"

        INCLUDE_DIRS
            "."

        REQUIRES
            homer_sensor
            homer_sensors
            homer_util
            nvs_flash
            esp_http_client
            esp_http_server
)
