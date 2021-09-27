idf_component_register(
        SRCS
            "homer_sensor.cpp"
            "homer_sensor.hpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            homer_util
)
