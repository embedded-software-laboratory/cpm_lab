cmake_minimum_required(VERSION 3.5)

option(BUILD_ARM OFF)

set (CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Werror=return-type)
include_directories(src)
include_directories(include)

file(GLOB SOURCE_DDS "dds_idl_cpp/*")

set(SOURCE_CPM
    include/cpm/CommandLineReader.hpp
    src/CommandLineReader.cpp
    include/cpm/get_topic.hpp
    include/cpm/Parameter.hpp
    include/cpm/ParameterReceiver.hpp
    src/ParameterReceiver.cpp
    include/cpm/ParticipantSingleton.hpp
    src/ParticipantSingleton.cpp
    include/cpm/Participant.hpp
    include/cpm/Reader.hpp
    include/cpm/ReaderAbstract.hpp
    include/cpm/Writer.hpp
    include/cpm/MultiVehicleReader.hpp
    include/cpm/Timer.hpp
    include/cpm/stamp_message.hpp
    src/Timer.cpp
    include/cpm/TimerFD.hpp
    src/TimerFD.cpp
    include/cpm/SimpleTimer.hpp
    src/SimpleTimer.cpp
    src/TimerSimulated.hpp
    src/TimerSimulated.cpp
    include/cpm/exceptions.hpp
    src/exceptions.cpp
    include/cpm/AsyncReader.hpp
    src/ParticipantSingleton.cpp
    include/cpm/ParticipantSingleton.hpp
    src/Logging.cpp
    include/cpm/Logging.hpp
    src/InternalConfiguration.hpp
    src/InternalConfiguration.cpp
    include/cpm/init.hpp
    include/cpm/get_time_ns.hpp
    src/get_time_ns.cpp
    include/cpm/RTTTool.hpp
    src/RTTTool.cpp
    include/cpm/TimeMeasurement.hpp
    src/TimeMeasurement.cpp
)



# Configuration for RTI Connext
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(dl nsl m pthread rt)
link_libraries(nddscpp2 nddsc nddscore)
include_directories(dds_idl_cpp)

if(BUILD_ARM)
    link_directories($ENV{NDDSHOME}/lib/armv6vfphLinux3.xgcc4.7.2)
    add_definitions(-fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls)
else()
    link_directories($ENV{NDDSHOME}/lib/x64Linux4gcc7.3.0)
    add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")
endif()


add_library(cpm SHARED
    ${SOURCE_DDS}
    ${SOURCE_CPM}
)


if(NOT BUILD_ARM)
    add_executable(unittest 
        test/catch.cpp
        test/test_logging.cpp
        test/test_AsyncReader.cpp
        test/test_rtt.cpp
        test/test_parameter.cpp
        test/test_simple_timer.cpp
        test/test_simple_timer_stop.cpp
        test/test_timer.cpp
        test/test_timer_stop.cpp
        test/test_timer_stop_running.cpp
        test/test_timer_start_again.cpp
        test/test_timer_simulated.cpp
        test/test_VehicleIDFilteredTopic.cpp
        test/test_Participant.cpp
        test/test_Reader.cpp
        test/test_ReaderAbstract.cpp
        test/test_Writer.cpp
        test/test_MultiVehicleReader.cpp
        test/test_CommandLineReader.cpp
        test/test_InternalConfiguration.cpp
    )

    target_link_libraries(unittest cpm)
endif()