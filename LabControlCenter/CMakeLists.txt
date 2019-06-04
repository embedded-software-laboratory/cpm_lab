cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_FLAGS_DEBUG  "-g")
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)
link_libraries(dl nsl m pthread rt)

# RTI DDS
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux3gcc5.4.0)
include_directories(src)
include_directories(.)
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")

# GTK
find_package(PkgConfig)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

# CPM lib
include_directories(SYSTEM ../../cpm_base/cpm_lib/include/)
include_directories(SYSTEM ../../cpm_base/cpm_lib/include/cpm/dds/)
link_directories(SYSTEM ../../cpm_base/cpm_lib/build/)

# yaml-cpp
include_directories(../../yaml-cpp/include/)
link_directories(../../yaml-cpp/build/)


add_executable(LabControlCenter
    src/TimeSeries.cpp
    src/TimeSeries.hpp
    src/TimeSeriesAggregator.cpp
    src/TimeSeriesAggregator.hpp
    src/VehicleManualControl.cpp
    src/VehicleManualControl.hpp
    ui/manual_control/VehicleManualControlUi.cpp
    ui/manual_control/VehicleManualControlUi.hpp
    ui/monitoring/MonitoringUi.cpp
    ui/monitoring/MonitoringUi.hpp
    ui/map_view/MapViewUi.cpp
    ui/map_view/MapViewUi.hpp
    ui/file_chooser/FileChooserUI.hpp
    ui/file_chooser/FileChooserUI.cpp
    ui/file_chooser/FileSaverUI.hpp
    ui/file_chooser/FileSaverUI.cpp
    ui/params/ParamViewUI.hpp
    ui/params/ParamViewUI.hpp
    ui/params/ParamViewUI.cpp
    ui/params/ParamsCreateView.hpp
    ui/params/ParamsCreateView.cpp
    ui/params/ParamModelRecord.hpp
    ui/right_tabs/TabsViewUI.hpp
    ui/right_tabs/TabsViewUI.cpp
    ui/timer/TimerViewUI.hpp
    ui/timer/TimerViewUI.cpp
    ui/MainWindow.cpp
    ui/MainWindow.hpp
    src/defaults.cpp
    src/defaults.hpp
    src/Point.hpp
    src/Joystick.cpp
    src/Joystick.hpp
    src/ParameterServer.cpp
    src/ParameterServer.hpp
    src/ParameterStorage.cpp
    src/ParameterStorage.hpp
    src/TrajectoryCommand.cpp
    src/TrajectoryCommand.hpp
    src/main.cpp
)

target_link_libraries(LabControlCenter cpm yaml-cpp ${GTKMM_LIBRARIES})
target_include_directories(LabControlCenter PUBLIC ${GTKMM_INCLUDE_DIRS})
target_compile_options(LabControlCenter PUBLIC ${GTKMM_CFLAGS_OTHER})
