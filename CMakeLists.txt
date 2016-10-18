cmake_minimum_required (VERSION 2.8)
project(VisualTracking)

include_directories (utils)
link_directories (${CMAKE_CURRENT_BINARY_DIR}/utils)

add_subdirectory(utils)

add_subdirectory(GraphCut)

add_subdirectory(KalmanFilter)

add_subdirectory(KCF)

add_subdirectory(LineDetect)

add_subdirectory(TreeDetect)