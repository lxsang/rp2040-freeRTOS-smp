cmake_minimum_required(VERSION 3.13)

add_library(on_either_core_common INTERFACE)
target_sources(on_either_core_common INTERFACE
        main_example.c)
target_include_directories(on_either_core_common INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(on_either_core_common INTERFACE
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap1
        pico_stdlib
        pico_multicore)

add_executable(rtos_on_core_zero)
target_link_libraries(rtos_on_core_zero on_either_core_common)
pico_add_extra_outputs(rtos_on_core_zero)
pico_enable_stdio_usb(rtos_on_core_zero 1)

add_executable(rtos_on_core_one)
target_link_libraries(rtos_on_core_one on_either_core_common)
target_compile_definitions(rtos_on_core_one PRIVATE
        mainRUN_FREE_RTOS_ON_CORE=1
        PICO_STACK_SIZE=0x1000
)

pico_add_extra_outputs(rtos_on_core_one)
pico_enable_stdio_usb(rtos_on_core_one 1)


