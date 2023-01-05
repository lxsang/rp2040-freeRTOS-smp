aux_source_directory(. DIR_examples_SRCS)
add_library(examples ${DIR_examples_SRCS})
target_link_libraries(examples PUBLIC waveshare_lcd_128 hardware_clocks hardware_rtc hardware_adc)