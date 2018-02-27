cmake_minimum_required(VERSION 3.5)
project(ros2_matlab_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cpm_tools REQUIRED)
find_package(cpm_msgs REQUIRED)

set(Matlab_ROOT_DIR /opt/MATLAB/R2017b)
find_package(Matlab)

include_directories(${Matlab_INCLUDE_DIRS})


message(WARNING ${Matlab_MEX_LIBRARY})
message(WARNING ${Matlab_INCLUDE_DIRS})

matlab_add_mex(NAME add_two_ints SRC src/add_two_ints.cpp)

matlab_add_mex(NAME receive_single_ros_message SRC src/receive_single_ros_message.cpp)
ament_target_dependencies(receive_single_ros_message rclcpp)
ament_target_dependencies(receive_single_ros_message cpm_msgs)
ament_target_dependencies(receive_single_ros_message cpm_tools)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS
  add_two_ints
  receive_single_ros_message
  DESTINATION lib/${PROJECT_NAME}
)
