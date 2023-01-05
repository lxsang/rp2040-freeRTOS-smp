cmake_minimum_required(VERSION 3.13)

add_executable(rtos_smp rtos_smp.c)
target_include_directories(rtos_smp PRIVATE ${CMAKE_CURRENT_LIST_DIR})
pico_enable_stdio_usb(rtos_smp 1)
pico_add_extra_outputs(rtos_smp)
target_link_libraries(rtos_smp PUBLIC
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap1
        waveshare_lcd_128
        hardware_clocks
        hardware_rtc
        pico_multicore
)