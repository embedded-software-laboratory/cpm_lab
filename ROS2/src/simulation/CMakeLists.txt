cmake_minimum_required(VERSION 3.5)
project(simulation)

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

add_executable(simple_simulation src/simple_simulation.cpp)
ament_target_dependencies(simple_simulation rclcpp)
ament_target_dependencies(simple_simulation cpm_msgs)
ament_target_dependencies(simple_simulation cpm_tools)

install(TARGETS
  simple_simulation
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
