cmake_minimum_required(VERSION 3.1)
add_definitions(-g -Wall -Wextra -Werror=return-type)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)
set (CMAKE_CXX_STANDARD 20)
project(DynamicPriorities)

# RTI DDS
add_definitions(-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT -DRTI_STATIC)
include_directories(SYSTEM $ENV{NDDSHOME}/include)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds)
include_directories(SYSTEM $ENV{NDDSHOME}/include/ndds/hpp)
link_libraries(nddscpp2z nddscz nddscorez)
link_directories($ENV{NDDSHOME}/lib/x64Linux4gcc7.3.0)
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -m64 -Wl,--no-as-needed")


# CPM lib
include_directories(SYSTEM ../software/cpm_lib/include/)
include_directories(SYSTEM ../software/cpm_lib/include/cpm/dds/)
link_directories(../software/cpm_lib/build/)


# exe
include_directories(src)
link_libraries(dl nsl m pthread rt)

include_directories(src/dds_idl_cpp)

file(GLOB dds_SRC CONFIGURE_DEPENDS
     "./include/dds/*.hpp"
     "./include/dds/*.h"
     "./src/dds_idl_cpp/*.cxx"
     "./src/dds_idl_cpp/*.hpp"
)

add_executable(dynamic_priorities
    src/main.cpp
    include/lane_graph_full/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
    src/VehicleTrajectoryPlanningState.hpp
    src/VehicleTrajectoryPlanningState.cpp
    src/VehicleTrajectoryPlanner.hpp
    src/VehicleTrajectoryPlanner.cpp
    src/CouplingGraph.cpp
    src/CouplingGraph.hpp
    ${dds_SRC}
)
target_include_directories(dynamic_priorities PUBLIC
    include/lane_graph_full
    src/dds_idl_cpp
    include/dds
    )
target_link_libraries(dynamic_priorities cpm)

add_executable(tests
    src/tests.cpp
    include/lane_graph_full/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
)
target_include_directories(tests PUBLIC
    include/lane_graph_full)
target_link_libraries(tests cpm)


add_executable(simulation
    src/simulation.cpp
    include/lane_graph_full/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
    src/VehicleTrajectoryPlanningState.hpp
    src/VehicleTrajectoryPlanningState.cpp
    src/VehicleTrajectoryPlanner.hpp
    src/VehicleTrajectoryPlanner.cpp
    src/CouplingGraph.cpp
    src/CouplingGraph.hpp
    ${dds_SRC})
target_include_directories(simulation PUBLIC
    include/lane_graph_full
    src/dds_idl_cpp
    include/dds
    )
target_link_libraries(simulation cpm)