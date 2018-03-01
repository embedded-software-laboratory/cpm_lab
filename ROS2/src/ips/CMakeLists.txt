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

include_directories(src)

set(PYLON_ROOT /opt/pylon5)
include_directories(${PYLON_ROOT}/include)
link_directories(${PYLON_ROOT}/lib64)
set(Pylon_LIBRARIES GenApi_gcc_v3_0_Basler_pylon_v5_0 GCBase_gcc_v3_0_Basler_pylon_v5_0 pylonbase pylonutility)

SET(${PROJECT_NAME}_LIB_SOURCES
    src/CameraParameters/CameraParameters.cpp
    src/CameraParameters/CameraParameters.hpp
    src/CameraWrapper/CameraWrapper.cpp
    src/CameraWrapper/CameraWrapper.hpp
)

add_library(${PROJECT_NAME}_lib STATIC ${${PROJECT_NAME}_LIB_SOURCES})
ament_target_dependencies(${PROJECT_NAME}_lib cpm_tools)



add_executable(${PROJECT_NAME}_unittest
    src/tools/unittest/main.cpp
    src/CameraParameters/CameraParameters_unittest.cpp
)
target_link_libraries(${PROJECT_NAME}_unittest ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_unittest cpm_tools)
ament_target_dependencies(${PROJECT_NAME}_unittest cpm_msgs)



add_executable(${PROJECT_NAME}_calculate_camera_pose_calibration src/tools/calculate_camera_pose_calibration/main.cpp )
target_link_libraries(${PROJECT_NAME}_calculate_camera_pose_calibration ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_calculate_camera_pose_calibration cpm_tools)
ament_target_dependencies(${PROJECT_NAME}_calculate_camera_pose_calibration cpm_msgs)



add_executable(${PROJECT_NAME}_validate_camera_calibration src/tools/validate_camera_calibration/main.cpp )
target_link_libraries(${PROJECT_NAME}_validate_camera_calibration ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_validate_camera_calibration cpm_tools)
ament_target_dependencies(${PROJECT_NAME}_validate_camera_calibration cpm_msgs)



add_executable(${PROJECT_NAME}_active_beacon_light_detection src/tools/active_beacon_light_detection/main.cpp )
target_link_libraries(${PROJECT_NAME}_active_beacon_light_detection ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_active_beacon_light_detection cpm_tools)
ament_target_dependencies(${PROJECT_NAME}_active_beacon_light_detection cpm_msgs)


install(TARGETS
  ${PROJECT_NAME}_unittest
  ${PROJECT_NAME}_calculate_camera_pose_calibration
  ${PROJECT_NAME}_validate_camera_calibration
  ${PROJECT_NAME}_active_beacon_light_detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
