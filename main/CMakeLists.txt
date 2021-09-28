# set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=clang-analyzer-core.uninitialized.Branch)

idf_component_register(
        SRCS

            "homer1.cpp"
            "my_init.hpp"
            "my_init.cpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            homer_sensor
            homer_sensors
            homer_util
            nvs_flash
)
