cmake_minimum_required(VERSION 3.3)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)

# MATLAB
find_package(Matlab REQUIRED)
message("-- found MATLAB in ${Matlab_ROOT_DIR}")

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


# exe
link_libraries(dl nsl m pthread rt)
include_directories(${CMAKE_SOURCE_DIR}/include)


# Go to formation // Matlab libraries
add_executable(
    go_to_formation 
    src/main.cpp 
    src/mgen_init.cpp
    src/go_to_formation.cpp
)
target_include_directories(
    go_to_formation PRIVATE 
    ${CMAKE_SOURCE_DIR}/../matlab/Coder/codegen/lib/planTrajectory
    ${Matlab_ROOT_DIR}/extern/include/
    ${Matlab_ROOT_DIR}/extern/include/shared_autonomous
    ${Matlab_ROOT_DIR}/extern/include/shared_robotics 
)

target_link_libraries(
    go_to_formation
    ${CMAKE_SOURCE_DIR}/../matlab/Coder/codegen/lib/planTrajectory/planTrajectory.a
    ${Matlab_ROOT_DIR}/bin/glnxa64/libmwautonomouscodegen.so
    ${Matlab_ROOT_DIR}/bin/glnxa64/libmwmorphop_binary_tbb.so
    ${Matlab_ROOT_DIR}/sys/os/glnxa64/libiomp5.so
    cpm
)