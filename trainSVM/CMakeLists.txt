cmake_minimum_required (VERSION 2.8)
project (trainSVM)

find_package (OpenCV REQUIRED)

include_directories(liblinear)
include_directories(liblinear/blas)
include_directories(../utils)

aux_source_directory(. DIR_SRCS)
aux_source_directory(liblinear LIBLINEAR)
aux_source_directory(liblinear/blas BLAS)

add_executable (trainSVM main.cpp ${DIR_SRCS} ${LIBLINEAR} ${BLAS})
target_link_libraries (trainSVM ${OpenCV_LIBS})