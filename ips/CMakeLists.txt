cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Werror=return-type)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")


# OpenCV
set(PYLON_ROOT /opt/opencv400)
include_directories(${PYLON_ROOT}/include/opencv4)
link_directories(${PYLON_ROOT}/lib)
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
link_libraries(dl nsl m pthread rt)
link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")



# CPM lib
include_directories(../../cpm_base/cpm_lib/include/)
include_directories(../../cpm_base/cpm_lib/include/cpm/dds/)
link_directories(../../cpm_base/cpm_lib/build/)


set(SOURCE
	src/Camera.cpp
	src/CameraFileWrapper.cpp
	src/CameraFileWrapper.h
	src/Camera.h
	src/CameraParameter.cpp
	src/CameraParameter.h
	src/CameraStream.cpp
	src/CameraStream.h
	src/CommunicationAPI.cpp
	src/CommunicationAPI.h
	src/CommunicationAPI_LED.cpp
	src/CommunicationAPI_LED.h
	src/Constants.h
	src/default.h
	src/DetectedObjectsBuffer.h
	src/DetectedObjects.cpp
	src/DetectedObjects.h
	src/Evaluation.cpp
	src/Evaluation.h
	src/FileReaderHelper.cpp
	src/FileReaderHelper.h
	src/GlobalDataHelper.cpp
	src/GlobalDataHelper.h
	src/IllustrationHelper.cpp
	src/IllustrationHelper.h
	src/ImageHolderLED.cpp
	src/ImageHolderLED.h
	src/ImageProcessingHelper.cpp
	src/ImageProcessingHelper.h
    src/RemainingPoints.h
    src/RemainingPoints.cpp
	src/SortedObjects.cpp
	src/SortedObjects.h
	src/VehicleMapping.cpp
	src/VehicleMapping.h
	src/Target.h
	src/TargetLED.cpp
	src/TargetLED.h
	src/ThreadSafeQueue.h
    src/VehicleMapping.h
    src/VehicleMapping.cpp
)

add_executable(ips
    ${SOURCE}
    src/main_live.cpp
)

target_link_libraries(ips ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} yaml-cpp cpm)
