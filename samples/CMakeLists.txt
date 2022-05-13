find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

FIND_PACKAGE( Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

FIND_PACKAGE(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ../src/*.cpp)
file(GLOB HEADER_FILES ../include/*.hpp )

list(APPEND LINK_LIBS 
    cbdetect
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}  
    ${CERES_LIBRARIES} 
    ${Sophus_LIBRARIES} 
    fmt
)

# add_executable(mono_lidar_calib_with_point_cons_hdl64 mono_lidar_calib_with_point_cons_hdl64.cpp ${SOURCE_FILES} ${HEADER_FILES})
# target_link_libraries(mono_lidar_calib_with_point_cons_hdl64 cbdetect ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${Sophus_LIBRARIES} fmt )

# add_executable(mono_lidar_calib_with_point_cons_vlp16 mono_lidar_calib_with_point_cons_vlp16.cpp ${SOURCE_FILES} ${HEADER_FILES})
# target_link_libraries(mono_lidar_calib_with_point_cons_vlp16 cbdetect ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${Sophus_LIBRARIES} fmt )

# add_executable(stereo_lidar_calib_with_point_cons_hdl64 stereo_lidar_calib_with_point_cons_hdl64.cpp ${SOURCE_FILES} ${HEADER_FILES})
# target_link_libraries(stereo_lidar_calib_with_point_cons_hdl64   ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${Sophus_LIBRARIES} fmt cbdetect)

# add_executable(huangpu128 huangpu128.cpp ${SOURCE_FILES} ${HEADER_FILES})
# target_link_libraries(huangpu128   ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${Sophus_LIBRARIES} fmt cbdetect)

add_executable(mono_lidar_calib mono_lidar_calib.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(mono_lidar_calib ${LINK_LIBS})

add_executable(stereo_lidar_calib stereo_lidar_calib.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(stereo_lidar_calib ${LINK_LIBS})
