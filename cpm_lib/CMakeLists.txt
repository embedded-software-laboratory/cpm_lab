cmake_minimum_required(VERSION 3.5)


set (CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Werror=return-type)
include_directories(src)
include_directories(include)




# Configuration for RTI Connext
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories($ENV{NDDSHOME}/include)
include_directories($ENV{NDDSHOME}/include/ndds)
include_directories($ENV{NDDSHOME}/include/ndds/hpp)
link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
link_libraries(dl nsl m pthread rt)
link_libraries(nddscpp2z nddscz nddscorez)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")



set(SOURCE_RTI
    build/rti/Parameter.hpp
    build/rti/Parameter.cxx
    build/rti/ParameterImpl.h
    build/rti/ParameterImpl.cxx
    build/rti/ParameterImplPlugin.h
    build/rti/ParameterImplPlugin.cxx
    build/rti/ParameterRequest.hpp
    build/rti/ParameterRequest.cxx
    build/rti/ParameterRequestImpl.h
    build/rti/ParameterRequestImpl.cxx
    build/rti/ParameterRequestImplPlugin.h
    build/rti/ParameterRequestImplPlugin.cxx
)


set(SOURCE_CPM
    include/myadd.hpp
    src/myadd.cpp
    include/cpm/Parameter.hpp
    src/ParameterDistribution.cpp
    src/ParameterStorage.hpp
    src/ParameterStorage.cpp
    src/Subscriber.hpp
)


add_library(cpm SHARED
    ${SOURCE_RTI}
    ${SOURCE_CPM}
)



add_executable(unittest test/catch.cpp
    test/test_myadd.cpp
)

target_link_libraries(unittest cpm)