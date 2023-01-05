aux_source_directory(. WS_LCD_SRC)


include_directories(../Config)

add_library(LCD ${WS_LCD_SRC})
target_link_libraries(LCD PUBLIC Config)