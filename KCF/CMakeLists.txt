cmake_minimum_required (VERSION 2.8)
project(KCF)

# 在环境变量中添加能搜索到CPackConfig.cmake的目录即可
find_package (OpenCV REQUIRED)

if (NOT WIN32)
	ADD_DEFINITIONS("-std=c++0x -O3")
endif (NOT WIN32)

include_directories (../utils)
include_directories (../utils_opencv)

include_directories(./src)
aux_source_directory(./src DIR_SRCS)

add_executable(KCF main.cpp ${DIR_SRCS})
target_link_libraries (KCF ${OpenCV_LIBS})