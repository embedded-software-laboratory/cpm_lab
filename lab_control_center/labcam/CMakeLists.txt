project("labcam")
set(CMAKE_BUILD_TYPE Release)

cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type -Wno-unknown-pragmas)
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake" CACHE STRING "Modules for CMake" FORCE)

# OpenCV
set(OpenCV_LIBS_IN imgproc)
find_package(OpenCV 4.0.0 REQUIRED ${OpenCV_LIBS_IN})
link_libraries(${OpenCV_LIBS})

# GStreamer
find_package(GStreamer 1.0 REQUIRED)
find_package(GLIB REQUIRED COMPONENTS)
message("-- found GLIB ${GLib}")
include_directories(${GSTREAMER_INCLUDE_DIRS})

# Pylon
find_package(Pylon REQUIRED)
include_directories(${PYLON_INCLUDE_DIRS})
message("-- found Pylon Version ${PYLON_VERSION}")

# CPM lib
link_directories(../cpm_lib/build/)

# LabCam Binary
FILE(GLOB SRC src/main.cpp)
add_executable(labcam_recorder ${SRC})

# LabCam Lib
include_directories("include")
FILE(GLOB SRC src/LabCam.cpp  src/LabCamIface.cpp src/CInstantCameraAppSrc.cpp)
add_library(labcamlib SHARED ${SRC})
target_link_libraries(labcamlib ${OpenCV_LIBRARIES} ${PYLON_LIBRARIES} ${GLIB_LIBRARIES} ${GSTREAMER_LIBRARIES} ${GLib_LIBDIR} gobject-2.0 pthread)
target_link_libraries(labcam_recorder cpm labcamlib)