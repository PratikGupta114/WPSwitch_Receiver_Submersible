# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

set(PROJECT_VER "1.0.0")

if(DEFINED ENV{BOARD_VARIANT})
    set(VARIANT $ENV{BOARD_VARIANT})
else()
    set(VARIANT "rev1")
endif()

message(STATUS "Building Submersible ControlUnit for BOARD_VARIANT: ${VARIANT}")

set(SDKCONFIG "${CMAKE_SOURCE_DIR}/sdkconfig.${VARIANT}")

if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.defaults")
    if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.defaults.${VARIANT}")
        set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.${VARIANT}")
    else()
        set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
    endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project("wpswitch-submersible-${VARIANT}")
