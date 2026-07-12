# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Board selection (set BOARD env var before building):
#   lcd_1_85    - Waveshare ESP32-S3-Touch-LCD-1.85  (default, custom BSP in boards/)
#   amoled_1_8  - Waveshare ESP32-S3-Touch-AMOLED-1.8 (registry BSP, see main/idf_component.yml)
if(NOT DEFINED ENV{BOARD})
    set(ENV{BOARD} "lcd_1_85")
endif()
message(STATUS "Building for board: $ENV{BOARD}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
    ./components/CANBus_Driver
    ./components/gps_locator
    )

if("$ENV{BOARD}" STREQUAL "lcd_1_85")
    list(APPEND EXTRA_COMPONENT_DIRS ./boards/bsp_ws_s3_touch_lcd_1_85)
elseif("$ENV{BOARD}" STREQUAL "amoled_1_8")
    # vendored copy of waveshare/esp32_s3_touch_amoled_1_8 v2.0.3
    # (patched: touch probe retries, no abort when touch is missing)
    list(APPEND EXTRA_COMPONENT_DIRS ./boards/esp32_s3_touch_amoled_1_8)
endif()

add_compile_options(-Wno-format)
project(lvgl_demo_v9)
