aux_source_directory(. DIR_examples_SRCS)

include_directories(../lib/Config)
include_directories(../lib/GUI)
include_directories(../lib/LCD)
include_directories(../lib/QMI8658)

add_library(examples ${DIR_examples_SRCS})
target_link_libraries(examples PUBLIC Config LCD QMI8658 hardware_clocks hardware_rtc hardware_adc)