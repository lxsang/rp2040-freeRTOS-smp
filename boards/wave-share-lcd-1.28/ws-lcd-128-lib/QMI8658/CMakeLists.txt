aux_source_directory(. WS_IMU_SRC)

include_directories(../Config)

add_library(QMI8658 ${WS_IMU_SRC})
target_link_libraries(QMI8658 PUBLIC Config)