cmake_minimum_required(VERSION 3.5)
project(vehicle_relay)

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

add_executable(vehicle_relay src/vehicle_relay.cpp)
ament_target_dependencies(vehicle_relay rclcpp)
ament_target_dependencies(vehicle_relay cpm_msgs)
ament_target_dependencies(vehicle_relay cpm_tools)

install(TARGETS
  vehicle_relay
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
