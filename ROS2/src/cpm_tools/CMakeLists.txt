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

set(${PROJECT_NAME}_SRCS
    src/cpm_tools/AbsoluteTimer.cpp
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SRCS})
ament_target_dependencies(${PROJECT_NAME} rclcpp)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} pthread)



ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)
