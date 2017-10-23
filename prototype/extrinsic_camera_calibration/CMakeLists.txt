cmake_minimum_required(VERSION 2.8)
find_package(PkgConfig)
find_package( OpenCV REQUIRED )
find_package (Threads)
pkg_search_module(apriltag REQUIRED apriltag)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Werror=return-type -Wall)


add_executable( extrinsic_camera_calibration main.cpp AprilTagDetector.cpp AprilTagDetector.h Parameter.h)
target_link_libraries( extrinsic_camera_calibration apriltag ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})