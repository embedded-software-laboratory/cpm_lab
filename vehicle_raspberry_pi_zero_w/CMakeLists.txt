cmake_minimum_required(VERSION 3.0)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)

add_definitions(-D_DEFAULT_SOURCE)

add_executable (spi spi.c bcm2835.c)

target_include_directories (spi PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(spi ${CMAKE_THREAD_LIBS_INIT} crypt m rt)