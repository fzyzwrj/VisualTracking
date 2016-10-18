cmake_minimum_required (VERSION 2.8)
project (GraphCut)

find_package (OpenCV REQUIRED)
include_directories (utils)

aux_source_directory (./src DIR_SRCS)

add_executable(GraphCut ${DIR_SRCS})
target_link_libraries (GraphCut ${OpenCV_LIBS} utils)