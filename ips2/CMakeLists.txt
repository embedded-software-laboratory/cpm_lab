cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type -Wno-unknown-pragmas)
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake" CACHE STRING "Modules for CMake" FORCE)

# OpenCV
set(OPENCV_ROOT /opt/opencv400)
include_directories(${OPENCV_ROOT}/include/opencv4)
link_directories(${OPENCV_ROOT}/lib)
set(OpenCV_LIBRARIES opencv_calib3d opencv_core opencv_features2d opencv_flann opencv_highgui opencv_imgcodecs opencv_imgproc opencv_ml opencv_objdetect opencv_photo opencv_stitching opencv_videoio opencv_video )

# Pylon
find_package(Pylon REQUIRED)
include_directories(${PYLON_INCLUDE_DIRS})
message("-- found Pylon Version ${PYLON_VERSION}")


# RTI DDS
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux4gcc7.3.0)
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")


# CPM lib
include_directories(SYSTEM ../../cpm_base/cpm_lib/include/)
include_directories(SYSTEM ../../cpm_base/cpm_lib/include/cpm/dds/)
link_directories(../../cpm_base/cpm_lib/build/)



include_directories(src)
link_libraries(dl nsl m pthread rt)



add_executable(BaslerLedDetection
    src/main_led_detection.cpp
)
target_link_libraries(BaslerLedDetection cpm ${OpenCV_LIBRARIES} ${PYLON_LIBRARIES})



add_executable(ips_pipeline
    src/main_ips_pipeline.cpp
    src/IpsPipeline.cpp
    src/IpsPipeline.hpp
    src/UndistortPoints.cpp
    src/UndistortPoints.hpp
    src/DetectVehicles.cpp
    src/DetectVehicles.hpp
    src/DetectVehicleID.cpp
    src/DetectVehicleID.hpp
    src/PoseCalculation.cpp
    src/PoseCalculation.hpp
)
target_link_libraries(ips_pipeline cpm ${OpenCV_LIBRARIES})


add_executable(unittest
    test/catch.cpp
    test/catch.hpp
    test/test_DetectVehicles.cpp
    test/test_DetectVehicleID.cpp
    src/DetectVehicles.cpp
    src/DetectVehicles.hpp
    src/DetectVehicleID.cpp
    src/DetectVehicleID.hpp
)
target_link_libraries(unittest cpm ${OpenCV_LIBRARIES})