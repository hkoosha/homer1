idf_component_register(
        SRCS
            "bmp180.cpp"
            "bmp180.hpp"
            "pms5003.cpp"
            "pms5003.hpp"
            "s8.cpp"
            "s8.hpp"
            "sht3x.cpp"
            "sht3x.hpp"
            "sgp30.cpp"
            "sgp30.hpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            homer_util
            homer_i2c
            homer_sensor
)
