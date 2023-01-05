aux_source_directory(. WS_CONF_SRC)
add_library(Config ${WS_CONF_SRC})
target_link_libraries(Config PUBLIC pico_stdlib hardware_spi hardware_i2c hardware_pwm hardware_adc)