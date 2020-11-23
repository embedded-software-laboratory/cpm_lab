cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 17)

# RTI DDS
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(dl nsl m pthread rt)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux4gcc7.3.0)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")

# CPM lib
include_directories(SYSTEM ../cpm_lib/include/)
include_directories(SYSTEM ../cpm_lib/include/cpm/dds/)
link_directories(../cpm_lib/build/)


include_directories(src)

set (SOURCES 
    src/Communication.hpp
    src/TypedCommunication.hpp
    src/TypedCommunication.cpp
)

add_executable( middleware
    src/main.cpp
    ${SOURCES}
)

target_link_libraries(middleware cpm)

add_executable(unittest 
    test/catch.cpp
    test/test_hlc_to_vehicle.cpp
    test/test_vehicle_to_middleware.cpp
    test/test_middleware_to_hlc.cpp
    test/test_vehicle_read.cpp
    ${SOURCES}
)

target_link_libraries(unittest cpm)