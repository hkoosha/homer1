#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

EXTRA_COMPONENT_DIRS += ${IDF_PATH}/examples/cxx/experimental/experimental_cpp_component

PROJECT_NAME := homer1

include $(IDF_PATH)/make/project.mk
