cmake_minimum_required (VERSION 2.8)
project(KalmanFilter)

find_package (OpenCV REQUIRED)

aux_source_directory(./src DIR_SRCS)

add_executable(KalmanFilter ${DIR_SRCS})
target_link_libraries (KalmanFilter ${OpenCV_LIBS})