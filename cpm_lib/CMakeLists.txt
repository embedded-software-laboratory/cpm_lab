cmake_minimum_required(VERSION 3.5)


option(BUILD_ARM OFF)

set (CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Werror=return-type)
include_directories(src)
include_directories(include)


if(BUILD_ARM)
    set(BUILD_DIR build_arm)
else()
    set(BUILD_DIR build)
endif()


set(SOURCE_RTI
    rti/Parameter.hpp
    rti/Parameter.cxx
    rti/ParameterImpl.h
    rti/ParameterImpl.cxx
    rti/ParameterImplPlugin.h
    rti/ParameterImplPlugin.cxx
    rti/ParameterRequest.hpp
    rti/ParameterRequest.cxx
    rti/ParameterRequestImpl.h
    rti/ParameterRequestImpl.cxx
    rti/ParameterRequestImplPlugin.h
    rti/ParameterRequestImplPlugin.cxx
)


set(SOURCE_CPM
    include/cpm/Parameter.hpp
    src/ParameterDistribution.cpp
    src/ParameterStorage.hpp
    src/ParameterStorage.cpp
    src/ParameterServer.hpp
    src/ParameterServer.cpp
    include/cpm/Timer.hpp
    src/Timer.cpp
    src/TimerFD.hpp
    src/TimerFD.cpp
    src/Subscriber.hpp
    src/ParticipantSingleton.cpp
    include/cpm/ParticipantSingleton.hpp
)



# Configuration for RTI Connext
include_directories($ENV{NDDSHOME}/include)
include_directories($ENV{NDDSHOME}/include/ndds)
include_directories($ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(dl nsl m pthread rt)
link_libraries(nddscpp2 nddsc nddscore)
include_directories(rti)

if(BUILD_ARM)
    link_directories($ENV{NDDSHOME}/lib/armv6vfphLinux3.xgcc4.7.2)
    add_definitions(-fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls)
else()
    link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
    add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")
endif()


add_library(cpm SHARED
    ${SOURCE_RTI}
    ${SOURCE_CPM}
)


if(NOT BUILD_ARM)
    add_executable(unittest 
        test/catch.cpp
        test/test_parameter.cpp
        test/test_timer.cpp
    )

    target_link_libraries(unittest cpm)
endif()