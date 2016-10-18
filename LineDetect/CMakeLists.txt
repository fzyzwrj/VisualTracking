cmake_minimum_required (VERSION 2.8)
project (LineDetect)

find_package (OpenCV REQUIRED)

aux_source_directory(./src DIR_SRCS)

add_executable (LineDetect ${DIR_SRCS})
target_link_libraries (LineDetect ${OpenCV_LIBS} utils)