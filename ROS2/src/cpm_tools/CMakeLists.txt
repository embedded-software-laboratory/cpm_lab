cmake_minimum_required(VERSION 3.5)
project(cpm_tools)

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

include_directories(include)
ament_export_include_directories(include)

#add_executable(talker src/talker.cpp)
#ament_target_dependencies(talker rclcpp std_msgs)
#
#add_executable(listener src/listener.cpp)
#ament_target_dependencies(listener rclcpp std_msgs)
#
#install(TARGETS
#  talker
#  listener
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)
