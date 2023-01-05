cmake_minimum_required(VERSION 3.13)

add_executable(rtos_either_core rtos_either_core.c)
target_include_directories(rtos_either_core PRIVATE ${CMAKE_CURRENT_LIST_DIR})
pico_enable_stdio_usb(rtos_either_core 1)
pico_add_extra_outputs(rtos_either_core)
target_link_libraries(rtos_either_core PUBLIC
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap1
        waveshare_lcd_128
        hardware_clocks
        hardware_rtc
        pico_multicore
)