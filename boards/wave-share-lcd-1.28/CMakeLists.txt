cmake_minimum_required(VERSION 3.13)

add_subdirectory(./common)
include_directories(${waveshare_lcd_128_SOURCE_DIR})

add_subdirectory(./example-no-rtos)
add_subdirectory(./example-rtos)