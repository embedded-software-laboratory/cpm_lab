cmake_minimum_required(VERSION 3.5)
project(ips)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cpm_tools REQUIRED)
find_package(cpm_msgs REQUIRED)

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)

add_executable(ips_unittest
    src/tools/unittest/main.cpp
    src/AprilTagDetector/AprilTagDetector.cpp
    src/AprilTagDetector/AprilTagDetector.hpp
)
ament_target_dependencies(ips_unittest apriltag rclcpp std_msgs cpm_tools cpm_msgs)


install(TARGETS
  ips_unittest
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
