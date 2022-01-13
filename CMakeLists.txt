cmake_minimum_required(VERSION 3.0)
project(stereo_lidar_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -O3")
include_directories("include/")
include_directories("libcbdetect/include/")

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

FIND_PACKAGE( Boost )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_subdirectory(samples)
add_subdirectory(test)
add_subdirectory(libcbdetect)