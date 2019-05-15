cmake_minimum_required(VERSION 3.1)

option(BUILD_SIMULATION OFF)

add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_C_STANDARD 99)

# RTI DDS
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
include_directories(SYSTEM rtidds)
include_directories(src)


# CPM lib
include_directories(../../cpm_base/cpm_lib/include/)
include_directories(../../cpm_base/cpm_lib/include/cpm/dds/)





set(SOURCE
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



if(BUILD_SIMULATION) # x64 options
    link_directories(../../cpm_base/cpm_lib/build/)

    link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
    add_executable(vehicle_rpi_firmware
        ${SOURCE}
        src/SimulationVehicle.cxx
        src/SimulationVehicle.hpp
        src/SimulationIPS.cxx
        src/SimulationIPS.hpp
    )

    target_compile_options(vehicle_rpi_firmware PUBLIC -fpic -DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -m64 -Wl,--no-as-needed)
    target_compile_options(vehicle_rpi_firmware PUBLIC -DVEHICLE_SIMULATION)
    SET_TARGET_PROPERTIES(vehicle_rpi_firmware PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
else() # ARM/Raspberry options
    link_directories(../../cpm_base/cpm_lib/build_arm/)

    link_directories($ENV{NDDSHOME}/lib/armv6vfphLinux3.xgcc4.7.2)
    add_executable(vehicle_rpi_firmware 
        ${SOURCE_RTI}
        ${SOURCE}
    )

    target_compile_options(vehicle_rpi_firmware PUBLIC -fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls)
endif()


target_link_libraries(vehicle_rpi_firmware dl nsl m pthread rt)
target_link_libraries(vehicle_rpi_firmware nddscpp2 nddsc nddscore)
target_link_libraries(vehicle_rpi_firmware cpm)