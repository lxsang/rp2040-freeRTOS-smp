cmake_minimum_required(VERSION 3.13)

add_executable(rtos_standard_full
        main.c
        main_full.c
        IntQueueTimer.c
        RegTest.s
        ../common/blocktim.c
        ../common/countsem.c
        ../common/dynamic.c
        ../common/recmutex.c
        ../common/QueueOverwrite.c
        ../common/EventGroupsDemo.c
        ../common/IntSemTest.c
        ../common/IntQueue.c
        ../common/TaskNotify.c
        ../common/TimerDemo.c
        ../common/GenQTest.c
        ../common/death.c
        ../common/semtest.c
        ../common/BlockQ.c
        ../common/flop.c
        )

target_compile_definitions(rtos_standard_full PRIVATE
        mainCREATE_SIMPLE_BLINKY_DEMO_ONLY=0
        )

target_include_directories(rtos_standard_full PRIVATE
        ${CMAKE_CURRENT_LIST_DIR})
include_directories(../common)
target_compile_definitions(rtos_standard_full PRIVATE
        PICO_STDIO_STACK_BUFFER_SIZE=64 # use a small printf on stack buffer
)
target_link_libraries(rtos_standard_full pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
pico_add_extra_outputs(rtos_standard_full)
pico_enable_stdio_usb(rtos_standard_full 1)

add_executable(rtos_standard_blinky
        main.c
        main_blinky.c
        )

target_compile_definitions(rtos_standard_blinky PRIVATE
        mainCREATE_SIMPLE_BLINKY_DEMO_ONLY=1
        )

target_include_directories(rtos_standard_blinky PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/Common/include)

target_link_libraries(rtos_standard_blinky pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap1)
pico_add_extra_outputs(rtos_standard_blinky)
pico_enable_stdio_usb(rtos_standard_blinky 1)