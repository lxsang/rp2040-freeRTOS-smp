aux_source_directory(. WS_GUI_SRC)


include_directories(../Config)

add_library(GUI ${WS_GUI_SRC})
target_link_libraries(GUI PUBLIC Config)