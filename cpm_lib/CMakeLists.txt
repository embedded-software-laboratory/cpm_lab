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


set(SOURCE_DDS
    dds/VehicleCommandDirectImpl.h
    dds/ParameterRequest.hpp
    dds/VehicleCommandTrajectoryImpl.h
    dds/TimeStamp.cxx
    dds/VehicleCommandTrajectory.hpp
    dds/VehicleCommandDirect.hpp
    dds/VehicleCommandDirectImpl.cxx
    dds/Pose2D.cxx
    dds/HeaderImpl.h
    dds/Pose2D.hpp
    dds/VehicleStateImpl.cxx
    dds/VehicleCommandSpeedCurvatureImplPlugin.h
    dds/VehicleCommandDirectImplPlugin.h
    dds/VehicleCommandSpeedCurvature.cxx
    dds/ParameterRequestImpl.h
    dds/Pose2DImpl.h
    dds/TimeStampImplPlugin.cxx
    dds/ParameterRequestImplPlugin.h
    dds/VehicleStateImplPlugin.cxx
    dds/ParameterRequestImpl.cxx
    dds/VehicleCommandTrajectoryImplPlugin.h
    dds/Header.hpp
    dds/HeaderImpl.cxx
    dds/ParameterImpl.h
    dds/ParameterRequest.cxx
    dds/ParameterImpl.cxx
    dds/VehicleState.hpp
    dds/Parameter.hpp
    dds/Pose2DImplPlugin.h
    dds/Header.cxx
    dds/VehicleCommandTrajectoryImplPlugin.cxx
    dds/TimeStampImpl.h
    dds/VehicleCommandSpeedCurvature.hpp
    dds/ParameterRequestImplPlugin.cxx
    dds/VehicleCommandTrajectory.cxx
    dds/HeaderImplPlugin.h
    dds/VehicleCommandSpeedCurvatureImpl.h
    dds/VehicleStateImplPlugin.h
    dds/Pose2DImpl.cxx
    dds/VehicleCommandTrajectoryImpl.cxx
    dds/TimeStamp.hpp
    dds/HeaderImplPlugin.cxx
    dds/Pose2DImplPlugin.cxx
    dds/VehicleStateImpl.h
    dds/VehicleCommandDirectImplPlugin.cxx
    dds/TimeStampImpl.cxx
    dds/ParameterImplPlugin.h
    dds/ParameterImplPlugin.cxx
    dds/VehicleCommandDirect.cxx
    dds/VehicleCommandSpeedCurvatureImplPlugin.cxx
    dds/TimeStampImplPlugin.h
    dds/VehicleState.cxx
    dds/VehicleCommandSpeedCurvatureImpl.cxx
    dds/Parameter.cxx
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