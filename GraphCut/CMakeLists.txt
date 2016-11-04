cmake_minimum_required (VERSION 2.8)
project (GraphCut)

find_package (OpenCV REQUIRED)

include_directories (../utils)
include_directories (../utils_opencv)
aux_source_directory (. DIR_SRCS)

add_executable(GraphCut ${DIR_SRCS} ../utils/utils.h ../utils_opencv/utils_opencv.h)
target_link_libraries (GraphCut ${OpenCV_LIBS})