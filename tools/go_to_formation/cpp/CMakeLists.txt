cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)

# RTI DDS
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux4gcc7.3.0)
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")


# CPM lib
include_directories(SYSTEM ../../../cpm_lib/include/)
include_directories(SYSTEM ../../../cpm_lib/include/cpm/dds/)
link_directories(../../../cpm_lib/build/)

# Matlab lib

add_library(planTrajectory SHARED IMPORTED)
set_target_properties(planTrajectory PROPERTIES IMPORTED_LOCATION "/home/vp/dev/software/tools/go_to_formation/cpp/matlab_dll/planTrajectory/planTrajectory.so")


# exe

link_libraries(dl nsl m pthread rt)

add_executable(mlib_test main.cpp)

target_link_libraries(mlib_test cpm)
target_link_libraries(mlib_test planTrajectory)




