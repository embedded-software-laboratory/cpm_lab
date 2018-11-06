cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)
link_libraries(dl nsl m pthread rt)
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
add_definitions(-O0 -g3) # for debugging!
include_directories($ENV{NDDSHOME}/include)
include_directories($ENV{NDDSHOME}/include/ndds)
include_directories($ENV{NDDSHOME}/include/ndds/hpp)
include_directories(rtidds)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
include_directories(src)
include_directories(.)
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")

find_package(PkgConfig)

pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
pkg_check_modules(WEBKIT2 REQUIRED webkit2gtk-4.0)

add_executable(LabControlCenter

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
    rtidds/Pose2D.cxx
    rtidds/Pose2D.hpp
    rtidds/Pose2DImpl.cxx
    rtidds/Pose2DImpl.h
    rtidds/Pose2DImplPlugin.cxx
    rtidds/Pose2DImplPlugin.h

    src/AbsoluteTimer.cpp
    src/AbsoluteTimer.hpp
    src/VehicleManualControl.cpp
    src/VehicleManualControl.hpp
    ui/manual_control/VehicleManualControlUi.cpp
    ui/manual_control/VehicleManualControlUi.hpp
    ui/monitoring/MonitoringUi.cpp
    ui/monitoring/MonitoringUi.hpp
    src/defaults.cpp
    src/defaults.hpp
    src/Joystick.cpp
    src/Joystick.hpp
    src/main.cpp
)

target_link_libraries(LabControlCenter ${GTKMM_LIBRARIES} ${WEBKIT2_LIBRARIES})
target_include_directories(LabControlCenter PUBLIC ${GTKMM_INCLUDE_DIRS} ${WEBKIT2_INCLUDE_DIRS})
target_compile_options(LabControlCenter PUBLIC ${GTKMM_CFLAGS_OTHER} ${WEBKIT2_CFLAGS_OTHER})
