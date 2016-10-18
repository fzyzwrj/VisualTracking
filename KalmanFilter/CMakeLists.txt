cmake_minimum_required (VERSION 2.8)
project(KalmanFilter)

find_package (OpenCV REQUIRED)
include_directories(./src)
aux_source_directory(./src DIR_SRCS)

add_executable(KalmanFilter main.cpp ${DIR_SRCS})
target_link_libraries (KalmanFilter ${OpenCV_LIBS} utils)