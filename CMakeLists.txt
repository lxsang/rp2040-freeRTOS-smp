cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)
include(freeRTOS_kernel_import.cmake)

project(freeRTOS_SMP_RP2040)
pico_sdk_init()

add_subdirectory(./boards)
