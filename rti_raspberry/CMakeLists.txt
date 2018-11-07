cmake_minimum_required(VERSION 3.1)

option(BUILD_SIMULATION OFF)

add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_C_STANDARD 99)

include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
include_directories(SYSTEM rtidds)
include_directories(src)



set(SOURCE_RTI
    rtidds/Pose2D.cxx
    rtidds/Pose2D.hpp
    rtidds/Pose2DImpl.cxx
    rtidds/Pose2DImpl.h
    rtidds/Pose2DImplPlugin.cxx
    rtidds/Pose2DImplPlugin.h
    rtidds/TimeStamp.cxx
    rtidds/TimeStamp.hpp
    rtidds/TimeStampImpl.cxx
    rtidds/TimeStampImpl.h
    rtidds/TimeStampImplPlugin.cxx
    rtidds/TimeStampImplPlugin.h
    rtidds/VehicleCommand.cxx
    rtidds/VehicleCommand.hpp
    rtidds/VehicleCommandImpl.cxx
    rtidds/VehicleCommandImpl.h
    rtidds/VehicleCommandImplPlugin.cxx
    rtidds/VehicleCommandImplPlugin.h
    rtidds/VehicleState.cxx
    rtidds/VehicleState.hpp
    rtidds/VehicleStateImpl.cxx
    rtidds/VehicleStateImpl.h
    rtidds/VehicleStateImplPlugin.cxx
    rtidds/VehicleStateImplPlugin.h
)

set(SOURCE
    src/AbsoluteTimer.cxx
    src/AbsoluteTimer.hpp
    src/bcm2835.c
    src/bcm2835.h
    src/main.cxx
    src/SensorCalibration.cxx
    src/SensorCalibration.hpp
    src/Localization.cxx
    src/Localization.hpp
    src/Controller.cxx
    src/Controller.hpp
    src/TrajectoryInterpolation.cxx
    src/TrajectoryInterpolation.hpp
    src/spi.c
    src/spi.h
    ../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.h
    ../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.c
)



if(BUILD_SIMULATION)

    link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
    add_executable(vehicle_rpi_firmware 
        ${SOURCE_RTI}
        ${SOURCE}
        src/Simulation.cxx
        src/Simulation.hpp
    )

    target_compile_options(vehicle_rpi_firmware PUBLIC -fpic -DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -m64 -Wl,--no-as-needed)
    target_compile_options(vehicle_rpi_firmware PUBLIC -DVEHICLE_SIMULATION)
    target_link_libraries(vehicle_rpi_firmware dl nsl m pthread rt)
    target_link_libraries(vehicle_rpi_firmware nddscpp2 nddsc nddscore)
    SET_TARGET_PROPERTIES(vehicle_rpi_firmware PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")

else()

    link_directories($ENV{NDDSHOME}/lib/armv6vfphLinux3.xgcc4.7.2)
    add_executable(vehicle_rpi_firmware 
        ${SOURCE_RTI}
        ${SOURCE}
    )

    target_compile_options(vehicle_rpi_firmware PUBLIC -fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls)
    target_link_libraries(vehicle_rpi_firmware dl nsl m pthread rt)
    target_link_libraries(vehicle_rpi_firmware nddscpp2 nddsc nddscore)

endif()