cmake_minimum_required(VERSION 3.13)
aux_source_directory(. waveshare_lcd_128_SOURCE_DIR)

add_subdirectory(./Config)
add_subdirectory(./LCD)
add_subdirectory(./Fonts)
add_subdirectory(./GUI)
add_subdirectory(./QMI8658)

add_library(waveshare_lcd_128 INTERFACE)
target_include_directories(waveshare_lcd_128 INTERFACE ./Config ./LCD ./Fonts ./GUI ./QMI8658)
target_link_libraries(waveshare_lcd_128 INTERFACE Config LCD Fonts GUI QMI8658)