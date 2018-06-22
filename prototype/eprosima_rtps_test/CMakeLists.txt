cmake_minimum_required(VERSION 3.5)
project(eprosima_rtps_test)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


include_directories(${CMAKE_CURRENT_BINARY_DIR}/idl)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idl)

add_custom_command(OUTPUT 
    idl/VehicleInput.cxx 
    idl/VehicleInputPubSubTypes.cxx
    COMMAND fastrtpsgen -replace ${CMAKE_CURRENT_SOURCE_DIR}/idl/VehicleInput.idl
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idl/
    VERBATIM
)

add_custom_command(OUTPUT 
    idl/VehicleState.cxx 
    idl/VehicleStatePubSubTypes.cxx
    COMMAND fastrtpsgen -replace ${CMAKE_CURRENT_SOURCE_DIR}/idl/VehicleState.idl
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idl/
    VERBATIM
)

add_executable(dynamics
    idl/VehicleInput.cxx
    idl/VehicleInputPubSubTypes.cxx
    idl/VehicleState.cxx
    idl/VehicleStatePubSubTypes.cxx
    dynamics.cpp
)

target_link_libraries(dynamics fastcdr fastrtps)


add_executable(controller
    idl/VehicleInput.cxx
    idl/VehicleInputPubSubTypes.cxx
    idl/VehicleState.cxx
    idl/VehicleStatePubSubTypes.cxx
    controller.cpp
)

target_link_libraries(controller fastcdr fastrtps)