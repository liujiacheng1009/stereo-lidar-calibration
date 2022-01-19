cmake_minimum_required(VERSION 3.0)
project(stereo_lidar_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -O3")
include_directories("include/")
include_directories("libcbdetect/include/")

add_subdirectory(samples)
# add_subdirectory(test)
add_subdirectory(libcbdetect)