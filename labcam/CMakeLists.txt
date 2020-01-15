project("CPM Lab Video Recorder")
set(CMAKE_BUILD_TYPE Release)

cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake" CACHE STRING "Modules for CMake" FORCE)

# GStreamer
find_package(GStreamer 1.0 REQUIRED)
find_package(GLIB REQUIRED COMPONENTS)
message("-- found GLIB ${GLib}")
include_directories(${GSTREAMER_INCLUDE_DIRS})

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

# BalserLedDetection
include_directories("include")
FILE(GLOB SRC src/*.cpp)
add_executable(recorder ${SRC})
target_link_libraries(recorder ${PYLON_LIBRARIES} ${GLIB_LIBRARIES} ${GSTREAMER_LIBRARIES} ${GLib_LIBDIR} gobject-2.0)
