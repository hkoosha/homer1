idf_component_register(
        SRCS
            "homer_i2c.cpp"
            "homer_i2c.hpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            homer_util
)
