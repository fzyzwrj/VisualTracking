cmake_minimum_required (VERSION 2.8)
project (TreeDetect)

find_package (OpenCV REQUIRED)

aux_source_directory(./src DIR_SRCS)

add_executable (TreeDetect ${DIR_SRCS})
target_link_libraries (TreeDetect ${OpenCV_LIBS})