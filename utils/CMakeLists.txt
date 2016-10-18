cmake_minimum_required (VERSION 2.8)
find_package (OpenCV REQUIRED)
aux_source_directory(. DIR_SRCS)
add_library(utils SHARED ${DIR_SRCS})

set_target_properties(utils PROPERTIES output_name "utils")