cmake_minimum_required(VERSION 3.0)
project(stereo_lidar_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -O3")
include_directories("include/")

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

FIND_PACKAGE( Boost )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR})

# Set header files 
set(
	HEADER_FILES
	include/utils.hpp
    include/extractChessboard.hpp
	include/extractImageFeature.hpp
	include/extractLidarFeature.hpp
	include/optimization.hpp
	include/matlab_data.hpp
	)

# Set source files
set(
	SOURCE_FILES 
    src/extractChessboard.cpp
	src/extractImageFeature.cpp
	src/extractLidarFeature.cpp 
	src/optimization.cpp
    )
# executable files
add_executable(test_extract_chessboard test/test_extract_chessboard.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(test_extract_chessboard ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_extract_image_feature test/test_extract_image_feature.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(test_extract_image_feature ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_extract_lidar_feature test/test_extract_lidar_feature.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(test_extract_lidar_feature ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_optimization test/test_optimization.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(test_optimization ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES})

add_executable(calibration_lc experiment/calibration_lc.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(calibration_lc ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES})

add_executable(calibration_lcc experiment/calibration_lcc.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(calibration_lcc ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_matlab_data test/test_matlab_data.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(test_matlab_data ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES})

add_executable(refine_matlab_results experiment/refine_matlab_results.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(refine_matlab_results ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES})