cmake_minimum_required(VERSION 3.1)


add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_C_STANDARD 99)
link_libraries(dl nsl m pthread rt)
add_definitions(-fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls)
include_directories($ENV{NDDSHOME}/include)
include_directories($ENV{NDDSHOME}/include/ndds)
include_directories($ENV{NDDSHOME}/include/ndds/hpp)
include_directories(rtidds)
include_directories(src)
link_libraries(nddscpp2 nddsc nddscore)

link_directories($ENV{NDDSHOME}/lib/armv6vfphLinux3.xgcc4.7.2)


add_executable(vehicle_rpi_firmware 

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
src/spi.c
src/spi.h

)

