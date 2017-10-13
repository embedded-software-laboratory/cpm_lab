cmake_minimum_required(VERSION 2.8)
find_package( OpenCV REQUIRED )
cmake_policy(SET CMP0015 NEW)
set(CMAKE_CXX_STANDARD 14)

include_directories(../../apriltag)
link_directories(../../apriltag)

add_executable( apriltag_track_video main.cpp )
target_link_libraries( apriltag_track_video ${OpenCV_LIBS} apriltag)