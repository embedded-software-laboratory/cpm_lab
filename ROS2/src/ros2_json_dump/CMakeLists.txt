cmake_minimum_required(VERSION 3.5)
project(ros2_json_dump)

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
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

add_executable(dump src/dump.cpp)
ament_target_dependencies(dump rclcpp)
ament_target_dependencies(dump cpm_msgs)
ament_target_dependencies(dump cpm_tools)
ament_target_dependencies(dump rosidl_typesupport_introspection_cpp)

install(TARGETS
  dump
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
