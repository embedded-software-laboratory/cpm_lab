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
link_directories($ENV{NDDSHOME}/lib/x64Linux4gcc7.3.0)
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
    src/LogStorage.hpp
    src/LogStorage.cpp
    src/TimerTrigger.hpp
    src/TimerTrigger.cpp
    src/TimeSeries.cpp
    src/TimeSeries.hpp
    src/TimeSeriesAggregator.cpp
    src/TimeSeriesAggregator.hpp
    src/HLCReadyAggregator.cpp
    src/HLCReadyAggregator.hpp
    src/VisualizationCommandsAggregator.cpp
    src/VisualizationCommandsAggregator.hpp
    src/VehicleManualControl.cpp
    src/VehicleManualControl.hpp
    ui/manual_control/VehicleManualControlUi.cpp
    ui/manual_control/VehicleManualControlUi.hpp
    src/VehicleAutomatedControl.cpp
    src/VehicleAutomatedControl.hpp
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
    ui/timer/TimerModelRecord.hpp
    ui/logger/LoggerViewUI.hpp
    ui/logger/LoggerViewUI.cpp
    ui/logger/LoggerModelRecord.hpp
    ui/setup/SetupViewUI.hpp
    ui/setup/SetupViewUI.cpp
    ui/setup/Deploy.hpp
    ui/setup/Deploy.cpp
    ui/setup/VehicleToggle.hpp
    ui/setup/VehicleToggle.cpp
    ui/setup/UploadWindow.hpp
    ui/setup/UploadWindow.cpp
    ui/MainWindow.cpp
    ui/MainWindow.hpp
    src/defaults.cpp
    src/defaults.hpp
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

add_executable(TimerTestSimulated
    test/TimerTestSimulated.cpp
)

target_link_libraries(TimerTestSimulated cpm)

add_executable(TimerTestRealtime
    test/TimerTestRealtime.cpp
)

target_link_libraries(TimerTestRealtime cpm)

add_executable(VisualizationTest
    test/VisualizationTest.cpp
)

target_link_libraries(VisualizationTest cpm)