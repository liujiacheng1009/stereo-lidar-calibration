cmake_minimum_required(VERSION 3.0)
project(stereo_lidar_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -O3")
include_directories("include/")

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Set header files 
set(
	HEADER_FILES
	include/utils.hpp
    include/extractChessboard.hpp
	include/extractImageFeature.hpp
	)

# Set source files
set(
	SOURCE_FILES 
    src/extractChessboard.cpp
	src/extractImageFeature.cpp
    )
# executable files
# add_executable(test_extract_chessboard test/test_extract_chessboard.cpp ${SOURCE_FILES} ${HEADER_FILES})
# target_link_libraries(test_extract_chessboard ${PCL_LIBRARIES})

add_executable(test_extract_image_feature test/test_extract_image_feature.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(test_extract_image_feature ${OpenCV_LIBS} ${PCL_LIBRARIES})