cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)


# OpenCV
set(OPENCV_ROOT /opt/opencv400)
include_directories(${OPENCV_ROOT}/include/opencv4)
link_directories(${OPENCV_ROOT}/lib)
set(OpenCV_LIBRARIES opencv_calib3d opencv_core opencv_dnn opencv_features2d opencv_flann opencv_gapi opencv_highgui opencv_imgcodecs opencv_imgproc opencv_ml opencv_objdetect opencv_photo opencv_stitching opencv_videoio opencv_video )



# Pylon
set(PYLON_ROOT /opt/pylon5)
include_directories(SYSTEM ${PYLON_ROOT}/include)
link_directories(${PYLON_ROOT}/lib64)
set(Pylon_LIBRARIES GenApi_gcc_v3_0_Basler_pylon_v5_0 GCBase_gcc_v3_0_Basler_pylon_v5_0 pylonbase pylonutility)


# RTI DDS
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
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
target_link_libraries(BaslerLedDetection cpm ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES})



add_executable(ips_manual_test
    src/main_ips_manual_test.cpp
    src/IpsPipeline.cpp
    src/IpsPipeline.hpp
    src/UndistortPoints.cpp
    src/UndistortPoints.hpp
    src/DetectVehicles.cpp
    src/DetectVehicles.hpp
    src/DetectVehicleID.cpp
    src/DetectVehicleID.hpp
)
target_link_libraries(ips_manual_test cpm ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES})


add_executable(unittest 
    test/catch.cpp
    test/test_DetectVehicles.cpp
    test/test_DetectVehicleID.cpp
    src/DetectVehicles.cpp
    src/DetectVehicleID.cpp
    src/DetectVehicleID.hpp
)
target_link_libraries(unittest cpm)