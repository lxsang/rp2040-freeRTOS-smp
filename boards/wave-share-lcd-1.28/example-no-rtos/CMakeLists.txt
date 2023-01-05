cmake_minimum_required(VERSION 3.13)

add_subdirectory(./examples)


include_directories(./examples)


add_executable(main_nortos main.c)

# enable usb output, disable uart output
pico_enable_stdio_usb(main_nortos 1)
pico_enable_stdio_uart(main_nortos 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main_nortos)

target_link_libraries(main_nortos examples LCD QMI8658 GUI Fonts Config pico_stdlib hardware_spi hardware_i2c )