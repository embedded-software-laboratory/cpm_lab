cmake_minimum_required(VERSION 3.1)

option(BUILD_ARM OFF)
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
include_directories(../cpm_lib/include/)
include_directories(../cpm_lib/include/cpm/dds/)





set(SOURCE_BASE
    src/main.cxx
    src/SensorCalibration.cxx
    src/SensorCalibration.hpp
    src/Localization.cxx
    src/Localization.hpp
    src/Controller.cxx
    src/Controller.hpp
    src/TrajectoryInterpolation.cxx
    src/TrajectoryInterpolation.hpp
    src/casadi_mpc_fn.c
    src/casadi_mpc_fn.h
    src/MpcController.hpp
    src/MpcController.cxx
    src/VehicleModel.hpp
    src/VehicleModel.cxx
    src/TimeMeasurement.hpp
    src/TimeMeasurement.cxx
    ../low_level_controller/vehicle_atmega2560_firmware/crc.h
    ../low_level_controller/vehicle_atmega2560_firmware/crc.c
)



set_source_files_properties(src/casadi_mpc_fn.c PROPERTIES COMPILE_FLAGS -O3)


if(BUILD_SIMULATION)
    set(SOURCE
        ${SOURCE_BASE}
        src/geometry.hpp
        src/SimulationVehicle.cxx
        src/SimulationVehicle.hpp
        src/SimulationIPS.cxx
        src/SimulationIPS.hpp
    )
else() 
    set(SOURCE
        ${SOURCE_BASE}
        src/bcm2835.c
        src/bcm2835.h
        src/spi.c
        src/spi.h
    )
endif()



if(BUILD_ARM)
    link_directories(../cpm_lib/build_arm/)
    link_directories($ENV{NDDSHOME}/lib/armv6vfphLinux3.xgcc4.7.2)
else()
    link_directories(../cpm_lib/build/)
    link_directories($ENV{NDDSHOME}/lib/x64Linux4gcc7.3.0)
endif()


add_executable(vehicle_rpi_firmware ${SOURCE})

if(BUILD_ARM)
    target_compile_options(vehicle_rpi_firmware PUBLIC -fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls)
else()
    target_compile_options(vehicle_rpi_firmware PUBLIC -fpic -DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -m64 -Wl,--no-as-needed)
    set_target_properties(vehicle_rpi_firmware PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
endif()


if(BUILD_SIMULATION)
    target_compile_options(vehicle_rpi_firmware PUBLIC -DVEHICLE_SIMULATION)
endif()


target_link_libraries(vehicle_rpi_firmware dl nsl m pthread rt)
target_link_libraries(vehicle_rpi_firmware nddscpp2 nddsc nddscore)
target_link_libraries(vehicle_rpi_firmware cpm)
