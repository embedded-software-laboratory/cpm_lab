cmake_minimum_required(VERSION 3.5)
project(ros2_matlab_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(Matlab_ROOT_DIR /opt/MATLAB/R2017b)
find_package(Matlab)

include_directories(${Matlab_INCLUDE_DIRS})



message(WARNING Hello world)
message(WARNING ${Matlab_MEX_LIBRARY})
message(WARNING ${Matlab_INCLUDE_DIRS})

matlab_add_mex(NAME add_two_ints SRC src/add_two_ints.cpp)

install(TARGETS
  add_two_ints
  DESTINATION lib/${PROJECT_NAME}
)
