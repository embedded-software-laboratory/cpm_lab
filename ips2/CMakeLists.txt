cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)


add_executable(BaslerLedDetection
    src/main_led_detection.cpp
)

#target_link_libraries(LabControlCenter cpm )
