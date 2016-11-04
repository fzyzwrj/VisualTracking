cmake_minimum_required (VERSION 2.8)
project (utils_test)

add_library(utils SHARED utils.cpp)
#add_library(utils_opencv utils_opencv.cpp)

add_executable(utils_test utils_test.cpp)
target_link_libraries(utils_test utils)
