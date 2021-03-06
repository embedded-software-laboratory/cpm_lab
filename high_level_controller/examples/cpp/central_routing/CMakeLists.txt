cmake_minimum_required(VERSION 3.1)
add_definitions(-Wall -Wextra -Werror=return-type)
set (CMAKE_CXX_STANDARD 11)

# RTI DDS
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux4gcc7.3.0)
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")


# CPM lib
include_directories(SYSTEM ../../../../cpm_lib/include/)
include_directories(SYSTEM ../../../../cpm_lib/include/cpm/dds/)
link_directories(../../../../cpm_lib/build/)


# exe
include_directories(src)
link_libraries(dl nsl m pthread rt)

add_executable(central_routing
    src/main.cpp
    include/lane_graph_full/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
    src/VehicleTrajectoryPlanningState.hpp
    src/VehicleTrajectoryPlanningState.cpp
    src/MultiVehicleTrajectoryPlanner.hpp
    src/MultiVehicleTrajectoryPlanner.cpp
)
target_include_directories(central_routing PUBLIC
    include/lane_graph_full)
target_link_libraries(central_routing cpm)

add_executable(central_routing_reduced
    src/main.cpp
    include/lane_graph_one_lane/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
    src/VehicleTrajectoryPlanningState.hpp
    src/VehicleTrajectoryPlanningState.cpp
    src/MultiVehicleTrajectoryPlanner.hpp
    src/MultiVehicleTrajectoryPlanner.cpp
)
target_include_directories(central_routing_reduced PUBLIC 
    include/lane_graph_one_lane)
target_link_libraries(central_routing_reduced cpm)

add_executable(150_jahre_RWTH_szenario
    src/main.cpp
    include/150_jahre_RWTH_szenario/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
    src/VehicleTrajectoryPlanningState.hpp
    src/VehicleTrajectoryPlanningState.cpp
    src/MultiVehicleTrajectoryPlanner.hpp
    src/MultiVehicleTrajectoryPlanner.cpp
)
target_include_directories(150_jahre_RWTH_szenario PUBLIC 
    include/150_jahre_RWTH_szenario)
target_link_libraries(150_jahre_RWTH_szenario cpm)

add_executable(outer_circle
    src/main.cpp
    include/lane_graph_outer_circle/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
    src/VehicleTrajectoryPlanningState.hpp
    src/VehicleTrajectoryPlanningState.cpp
    src/MultiVehicleTrajectoryPlanner.hpp
    src/MultiVehicleTrajectoryPlanner.cpp
)

target_include_directories(outer_circle PUBLIC 
    include/lane_graph_outer_circle)
target_link_libraries(outer_circle cpm)

add_executable(tests
    src/tests.cpp
    include/lane_graph_full/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
)
target_include_directories(tests PUBLIC
    include/lane_graph_full)
target_link_libraries(tests cpm)
