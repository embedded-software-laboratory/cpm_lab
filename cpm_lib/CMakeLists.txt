cmake_minimum_required(VERSION 3.5)

option(BUILD_ARM OFF)

set (CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Werror=return-type)
include_directories(src)
include_directories(include)

set(SOURCE_DDS
    dds/Header.cxx
    dds/Header.hpp
    dds/HeaderImpl.cxx
    dds/HeaderImpl.h
    dds/HeaderImplPlugin.cxx
    dds/HeaderImplPlugin.h
    dds/Parameter.cxx
    dds/Parameter.hpp
    dds/ParameterImpl.cxx
    dds/ParameterImpl.h
    dds/ParameterImplPlugin.cxx
    dds/ParameterImplPlugin.h
    dds/ParameterRequest.cxx
    dds/ParameterRequest.hpp
    dds/ParameterRequestImpl.cxx
    dds/ParameterRequestImpl.h
    dds/ParameterRequestImplPlugin.cxx
    dds/ParameterRequestImplPlugin.h
    dds/Pose2D.cxx
    dds/Pose2D.hpp
    dds/Pose2DImpl.cxx
    dds/Pose2DImpl.h
    dds/Pose2DImplPlugin.cxx
    dds/Pose2DImplPlugin.h
    dds/TimeStamp.cxx
    dds/TimeStamp.hpp
    dds/TimeStampImpl.cxx
    dds/TimeStampImpl.h
    dds/TimeStampImplPlugin.cxx
    dds/TimeStampImplPlugin.h
    dds/VehicleCommandDirect.cxx
    dds/VehicleCommandDirect.hpp
    dds/VehicleCommandDirectImpl.cxx
    dds/VehicleCommandDirectImpl.h
    dds/VehicleCommandDirectImplPlugin.cxx
    dds/VehicleCommandDirectImplPlugin.h
    dds/VehicleCommandSpeedCurvature.cxx
    dds/VehicleCommandSpeedCurvature.hpp
    dds/VehicleCommandSpeedCurvatureImpl.cxx
    dds/VehicleCommandSpeedCurvatureImpl.h
    dds/VehicleCommandSpeedCurvatureImplPlugin.cxx
    dds/VehicleCommandSpeedCurvatureImplPlugin.h
    dds/VehicleCommandTrajectory.cxx
    dds/VehicleCommandTrajectory.hpp
    dds/VehicleCommandTrajectoryImpl.cxx
    dds/VehicleCommandTrajectoryImpl.h
    dds/VehicleCommandTrajectoryImplPlugin.cxx
    dds/VehicleCommandTrajectoryImplPlugin.h
    dds/VehicleState.cxx
    dds/VehicleState.hpp
    dds/VehicleStateImpl.cxx
    dds/VehicleStateImpl.h
    dds/VehicleStateImplPlugin.cxx
    dds/VehicleStateImplPlugin.h
)


set(SOURCE_CPM
    include/cpm/Parameter.hpp
    src/ParameterDistribution.cpp
    src/ParameterStorage.hpp
    src/ParameterStorage.cpp
    src/ParameterServer.hpp
    src/ParameterServer.cpp
    include/cpm/ParticipantSingleton.hpp
    src/ParticipantSingleton.cpp
    include/cpm/Timer.hpp
    src/Timer.cpp
    src/TimerFD.hpp
    src/TimerFD.cpp
    src/Subscriber.hpp
)



# Configuration for RTI Connext
include_directories($ENV{NDDSHOME}/include)
include_directories($ENV{NDDSHOME}/include/ndds)
include_directories($ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(dl nsl m pthread rt)
link_libraries(nddscpp2 nddsc nddscore)
include_directories(dds)

if(BUILD_ARM)
    link_directories($ENV{NDDSHOME}/lib/armv6vfphLinux3.xgcc4.7.2)
    add_definitions(-fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls)
else()
    link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
    add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")
endif()


add_library(cpm SHARED
    ${SOURCE_DDS}
    ${SOURCE_CPM}
)


if(NOT BUILD_ARM)
    add_executable(unittest test/catch.cpp
        test/test_parameter.cpp
        test/test_timer.cpp
        test/test_VehicleIDFilteredTopic.cpp
    )

    target_link_libraries(unittest cpm)
endif()