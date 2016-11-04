cmake_minimum_required (VERSION 2.8)

project (utils_opencv_test)
find_package (OpenCV REQUIRED)
add_library (utils_opencv utils_opencv.cpp)

add_executable (utils_opencv_test utils_opencv_test.cpp test.jpg)

target_link_libraries (utils_opencv_test utils_opencv ${OpenCV_LIBS})