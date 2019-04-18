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
	VehicleDetection/Camera.cpp
	VehicleDetection/CameraFileWrapper.cpp
	VehicleDetection/CameraFileWrapper.h
	VehicleDetection/Camera.h
	VehicleDetection/CameraParameter.cpp
	VehicleDetection/CameraParameter.h
	VehicleDetection/CameraStream.cpp
	VehicleDetection/CameraStream.h
	VehicleDetection/CommunicationAPI.cpp
	VehicleDetection/CommunicationAPI.h
	VehicleDetection/CommunicationAPI_LED.cpp
	VehicleDetection/CommunicationAPI_LED.h
	VehicleDetection/Constants.h
	VehicleDetection/default.h
	VehicleDetection/DetectedObjectsBuffer.h
	VehicleDetection/DetectedObjects.cpp
	VehicleDetection/DetectedObjects.h
	VehicleDetection/Evaluation.cpp
	VehicleDetection/Evaluation.h
	VehicleDetection/FileReaderHelper.cpp
	VehicleDetection/FileReaderHelper.h
	VehicleDetection/GlobalDataHelper.cpp
	VehicleDetection/GlobalDataHelper.h
	VehicleDetection/IllustrationHelper.cpp
	VehicleDetection/IllustrationHelper.h
	VehicleDetection/ImageHolderLED.cpp
	VehicleDetection/ImageHolderLED.h
	VehicleDetection/ImageProcessingHelper.cpp
	VehicleDetection/ImageProcessingHelper.h
    VehicleDetection/RemainingPoints.h
    VehicleDetection/RemainingPoints.cpp
	VehicleDetection/SortedObjects.cpp
	VehicleDetection/SortedObjects.h
	VehicleDetection/VehicleMapping.cpp
	VehicleDetection/VehicleMapping.h
	VehicleDetection/Target.h
	VehicleDetection/TargetLED.cpp
	VehicleDetection/TargetLED.h
	VehicleDetection/ThreadSafeQueue.h
    VehicleDetection/VehicleMapping.h
    VehicleDetection/VehicleMapping.cpp
)






#add_executable(VehicleDetection
#    ${SOURCE}
#    VehicleDetection/main.cpp
#)

#target_link_libraries(VehicleDetection ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} yaml-cpp)



add_executable(VehicleDetection_live
    ${SOURCE}
    VehicleDetection/main_live.cpp
)

target_link_libraries(VehicleDetection_live ${OpenCV_LIBRARIES} ${Pylon_LIBRARIES} yaml-cpp cpm)
