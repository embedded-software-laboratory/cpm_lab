cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)

include_directories(src)

add_executable(LabControlCenter
    src/Joystick.hpp
    src/defaults.hpp
    src/Joystick.cpp
    src/main.cpp
)