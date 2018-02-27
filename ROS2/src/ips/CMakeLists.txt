cmake_minimum_required(VERSION 3.5)
project(ips)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

include_directories(/opt/opencv331/include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cpm_tools REQUIRED)
find_package(cpm_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV 3.3.1 EXACT REQUIRED)

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)

include_directories(src)

set(PYLON_ROOT /opt/pylon5)
include_directories(${PYLON_ROOT}/include)
link_directories(${PYLON_ROOT}/lib64)
set(Pylon_LIBRARIES GenApi_gcc_v3_0_Basler_pylon_v5_0 GCBase_gcc_v3_0_Basler_pylon_v5_0 pylonbase pylonutility)

SET(${PROJECT_NAME}_LIB_SOURCES
    src/AprilTagDetector/AprilTagDetector.cpp
    src/AprilTagDetector/AprilTagDetector.hpp
    src/CameraParameters/CameraParameters.cpp
    src/CameraParameters/CameraParameters.hpp
    src/DetectionDispatcherLogic/DetectionDispatcherLogic.cpp
    src/DetectionDispatcherLogic/DetectionDispatcherLogic.hpp
    src/CameraWrapper/CameraWrapper.cpp
    src/CameraWrapper/CameraWrapper.hpp
)

add_library(${PROJECT_NAME}_lib STATIC ${${PROJECT_NAME}_LIB_SOURCES})
ament_target_dependencies(${PROJECT_NAME}_lib cpm_tools)



add_executable(${PROJECT_NAME}_unittest
    src/tools/unittest/main.cpp
    src/AprilTagDetector/AprilTagDetector_unittest.cpp
    src/CameraParameters/CameraParameters_unittest.cpp
    src/DetectionDispatcherLogic/DetectionDispatcherLogic_unittest.cpp
)
target_link_libraries(${PROJECT_NAME}_unittest ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} apriltag yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_unittest cpm_tools)
ament_target_dependencies(${PROJECT_NAME}_unittest cpm_msgs)



add_executable(${PROJECT_NAME}_detection_simple_demo src/tools/detection_simple_demo/main.cpp)
target_link_libraries(${PROJECT_NAME}_detection_simple_demo ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} apriltag yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_detection_simple_demo cpm_tools)
ament_target_dependencies(${PROJECT_NAME}_detection_simple_demo cpm_msgs)



add_executable(${PROJECT_NAME}_calculate_camera_pose_calibration src/tools/calculate_camera_pose_calibration/main.cpp )
target_link_libraries(${PROJECT_NAME}_calculate_camera_pose_calibration ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} apriltag yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_calculate_camera_pose_calibration cpm_tools)
ament_target_dependencies(${PROJECT_NAME}_calculate_camera_pose_calibration cpm_msgs)



add_executable(${PROJECT_NAME}_validate_camera_calibration src/tools/validate_camera_calibration/main.cpp )
target_link_libraries(${PROJECT_NAME}_validate_camera_calibration ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} apriltag yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_validate_camera_calibration cpm_tools)
ament_target_dependencies(${PROJECT_NAME}_validate_camera_calibration cpm_msgs)




install(TARGETS
  ${PROJECT_NAME}_unittest
  ${PROJECT_NAME}_detection_simple_demo
  ${PROJECT_NAME}_calculate_camera_pose_calibration
  ${PROJECT_NAME}_validate_camera_calibration
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
