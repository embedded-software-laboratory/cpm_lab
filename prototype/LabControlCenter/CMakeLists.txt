cmake_minimum_required(VERSION 3.5)
project(LabControlCenter)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


find_package(wxWidgets REQUIRED COMPONENTS base net core gl html xml media qa adv xrc aui richtext)
include(${wxWidgets_USE_FILE})

add_executable(LabControlCenter
    ui/noname.cpp
    ui/noname.h
    main.cpp
)

target_link_libraries(LabControlCenter ${wxWidgets_LIBRARIES})