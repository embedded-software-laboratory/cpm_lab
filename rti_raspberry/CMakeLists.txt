cmake_minimum_required(VERSION 3.1)


add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_C_STANDARD 99)
link_libraries(dl nsl m pthread rt)
add_definitions(-fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls -DRTI_STATIC)
include_directories($ENV{NDDSHOME}/include)
include_directories($ENV{NDDSHOME}/include/ndds)
include_directories($ENV{NDDSHOME}/include/ndds/hpp)
include_directories(rtidds)
include_directories(src)
link_libraries(nddscpp2z nddscz nddscorez )

link_directories($ENV{NDDSHOME}/lib/armv6vfphLinux3.xgcc4.7.2)


add_executable(vehicle_rpi_firmware 

rtidds/TimeStamp.cxx
rtidds/Pose2D.cxx
rtidds/Pose2D.hpp
rtidds/VehicleStateImpl.cxx
rtidds/Pose2DImpl.h
rtidds/TimeStampImplPlugin.cxx
rtidds/VehicleStateImplPlugin.cxx
rtidds/VehicleState.hpp
rtidds/Pose2DImplPlugin.h
rtidds/TimeStampImpl.h
rtidds/VehicleStateImplPlugin.h
rtidds/Pose2DImpl.cxx
rtidds/TimeStamp.hpp
rtidds/Pose2DImplPlugin.cxx
rtidds/VehicleStateImpl.h
rtidds/TimeStampImpl.cxx
rtidds/TimeStampImplPlugin.h
rtidds/VehicleState.cxx

src/bcm2835.h
src/bcm2835.c
src/spi.h
src/spi.c
src/AbsoluteTimer.hpp
src/main.cxx
src/AbsoluteTimer.cxx

)

