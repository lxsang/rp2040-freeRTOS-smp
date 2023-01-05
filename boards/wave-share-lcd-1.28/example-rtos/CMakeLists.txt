cmake_minimum_required(VERSION 3.13)

add_executable(lcd_rtos lcd_example.c)
target_include_directories(lcd_rtos PRIVATE ${CMAKE_CURRENT_LIST_DIR})
pico_enable_stdio_usb(lcd_rtos 1)
pico_add_extra_outputs(lcd_rtos)
target_link_libraries(lcd_rtos PUBLIC
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap1
        waveshare_lcd_128
        hardware_clocks
        hardware_rtc
        pico_multicore
)