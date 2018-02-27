cmake_minimum_required(VERSION 3.5)
project(ips)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(/opt/opencv331/include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cpm_tools REQUIRED)
find_package(cpm_msgs REQUIRED)
find_package(OpenCV 3.3.1 EXACT REQUIRED)

#message(FATAL_ERROR ${cpm_tools_LIBRARIES})


find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)

include_directories(src)


set(PYLON_ROOT /opt/pylon5)
include_directories(${PYLON_ROOT}/include)
link_directories(${PYLON_ROOT}/lib64)

add_executable(ips_unittest
    src/tools/unittest/main.cpp
    src/AprilTagDetector/AprilTagDetector.cpp
    src/AprilTagDetector/AprilTagDetector.hpp
    src/AprilTagDetector/AprilTagDetector_unittest.cpp
    src/CameraParameters/CameraParameters.cpp
    src/CameraParameters/CameraParameters.hpp
    src/DetectionDispatcherLogic/DetectionDispatcherLogic.cpp
    src/DetectionDispatcherLogic/DetectionDispatcherLogic.hpp
    src/DetectionDispatcherLogic/DetectionDispatcherLogic_unittest.cpp
    src/CameraWrapper/CameraWrapper.cpp
    src/CameraWrapper/CameraWrapper.hpp
)
target_link_libraries(ips_unittest 
    yaml-cpp 
    ${OpenCV_LIBRARIES}
    pylonbase
    pylonutility
    apriltag
    GenApi_gcc_v3_0_Basler_pylon_v5_0 
    GCBase_gcc_v3_0_Basler_pylon_v5_0
)
ament_target_dependencies(ips_unittest std_msgs)
ament_target_dependencies(ips_unittest cpm_tools)
ament_target_dependencies(ips_unittest cpm_msgs)

install(TARGETS
  ips_unittest
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
