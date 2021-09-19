idf_component_register(
        SRCS

            "homer1.cpp"
            "my_init.hpp"
            "my_init.cpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            homer_sensor
            homer_util
            nvs_flash
)
