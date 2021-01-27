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
# add_library(planTrajectory SHARED IMPORTED)
# set_target_properties(planTrajectory PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/matlab_dll/planTrajectory/planTrajectory.so)
# Needs to be changed
add_library(iomp5 SHARED IMPORTED)
set_target_properties(iomp5 PROPERTIES IMPORTED_LOCATION /usr/local/MATLAB/R2020b/sys/os/glnxa64/libiomp5.so)

# exe
link_libraries(dl nsl m pthread rt)


# MVP: Get example to build
add_executable(mgen_example 
    ${CMAKE_SOURCE_DIR}/../matlab/Coder/codegen/lib/planTrajectory/examples/main.cpp
)
target_link_libraries(mgen_example ${CMAKE_SOURCE_DIR}/../matlab/Coder/codegen/lib/planTrajectory/planTrajectory.a)
target_include_directories(mgen_example PRIVATE 
    ${CMAKE_SOURCE_DIR}/../matlab/Coder/codegen/lib/planTrajectory/examples/
    ${CMAKE_SOURCE_DIR}/../matlab/Coder/codegen/lib/planTrajectory
    /usr/local/MATLAB/R2020b/extern/include
)

target_link_libraries(mgen_example cpm)
target_link_libraries(mgen_example iomp5)


# Go to formation
add_executable(mlib_test main.cpp)
target_link_libraries(mlib_test ${CMAKE_SOURCE_DIR}/../matlab/Coder/codegen/lib/planTrajectory/planTrajectory.a)
target_include_directories(mlib_test PRIVATE 
    ${CMAKE_SOURCE_DIR}/../matlab/Coder/codegen/lib/planTrajectory
    # /usr/local/MATLAB/R2020b/extern/include
)

target_link_libraries(mlib_test cpm)
target_link_libraries(mlib_test iomp5)
# target_link_libraries(mlib_test planTrajectory)
#target_include_directories(mlib_test PRIVATE ${CMAKE_SOURCE_DIR}/matlab_dll/planTrajectory)

