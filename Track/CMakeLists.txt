cmake_minimum_required (VERSION 2.8)
project(Track)

find_package (OpenCV REQUIRED)

#include_directories (../GraphCut/src)
include_directories (../KalmanFilter/src)
include_directories (../KCF/src)
include_directories (../LineDetect/src)
include_directories (../TreeDetect/src)


#aux_source_directory (../GraphCut/src)
aux_source_directory (../KalmanFilter/src DIR_KALMAN_FILTER_SRCS)
aux_source_directory (../KCF/src DIR_KCF_SRCS)
aux_source_directory (../LineDetect/src DIR_LINE_DETECT_SRCS)
aux_source_directory (../TreeDetect/src DIR_TREE_DETECT_SRCS)


add_executable(Track main.cpp ${DIR_KALMAN_FILTER_SRCS} ${DIR_KCF_SRCS} ${DIR_LINE_DETECT_SRCS} ${DIR_TREE_DETECT_SRCS})
target_link_libraries (Track ${OpenCV_LIBS} utils)