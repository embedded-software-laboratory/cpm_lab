cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)

include_directories(src)

# Pylon
set(PYLON_ROOT /opt/pylon5)
include_directories(SYSTEM ${PYLON_ROOT}/include)
link_directories(${PYLON_ROOT}/lib64)
set(Pylon_LIBRARIES GenApi_gcc_v3_0_Basler_pylon_v5_0 GCBase_gcc_v3_0_Basler_pylon_v5_0 pylonbase pylonutility)


# OpenCV
set(PYLON_ROOT /opt/opencv400)
include_directories(SYSTEM ${PYLON_ROOT}/include/opencv4)
link_directories(${PYLON_ROOT}/lib)
set(OpenCV_LIBRARIES opencv_calib3d opencv_core opencv_dnn opencv_features2d opencv_flann opencv_gapi opencv_highgui opencv_imgcodecs opencv_imgproc opencv_ml opencv_objdetect opencv_photo opencv_stitching opencv_videoio opencv_video )


add_executable(ips
    src/CameraWrapper.cpp
    src/CameraWrapper.hpp
    src/main.cpp
)


target_link_libraries(ips ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES})
