cmake_minimum_required(VERSION 3.5)

project(cpm_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(cpm_msgs
  msg/VehicleSensors.msg
  msg/VehicleObservation.msg
  msg/VehicleState.msg
  msg/ComplexTestMsg.msg
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs
)

ament_package()