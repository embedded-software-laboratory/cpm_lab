cmake_minimum_required(VERSION 3.0)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)

add_definitions(-D_DEFAULT_SOURCE)

add_executable (cpm_rpi_exe 
    main.c
    spi.c 
    bcm2835.c
    udp_tmp.c
)
target_include_directories (cpm_rpi_exe PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cpm_rpi_exe ${CMAKE_THREAD_LIBS_INIT} crypt m rt)