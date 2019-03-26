cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)
link_libraries(dl nsl m pthread rt)

# RTI DDS
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories($ENV{NDDSHOME}/include)
include_directories($ENV{NDDSHOME}/include/ndds)
include_directories($ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
include_directories(src)
include_directories(.)
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")

# CPM lib
include_directories(../cpm_base/cpm_lib/include/)
include_directories(../cpm_base/cpm_lib/include/cpm/dds/)
link_directories(../cpm_base/cpm_lib/build/)

# yaml-cpp
find_package(yaml-cpp PATHS ./yaml-cpp/)
include_directories(./yaml-cpp/include/)
link_directories(./yaml-cpp/build/)

add_executable(ParamServer
    src/Subscriber.hpp
    src/ParameterStorage.hpp
    src/ParameterStorage.cpp
    src/ParameterServer.hpp
    src/ParameterServer.cpp
    src/main.cpp
)

target_link_libraries(ParamServer cpm yaml-cpp)

add_executable(unittest
    src/ParameterStorage.hpp
    src/ParameterStorage.cpp
    test/catch.cpp
    test/catch.hpp
    test/parameterTest.cpp
)

target_link_libraries(unittest cpm yaml-cpp)