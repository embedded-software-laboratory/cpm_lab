
add_definitions(-Wall -std=c++11)
link_libraries(dl nsl m pthread rt)
add_definitions(-fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls -DRTI_STATIC)
include_directories(/home/janis/rti_connext_dds-5.3.1/include)
include_directories(/home/janis/rti_connext_dds-5.3.1/include/ndds)
link_libraries(nddscppz nddscz nddscorez )

link_directories(/home/janis/rti_connext_dds-5.3.1/lib/armv6vfphLinux3.xgcc4.7.2)


add_executable(hello_publisher 
    hello.cxx 
    hello_publisher.cxx 
    helloPlugin.cxx 
    helloSupport.cxx)

