
add_definitions(-Wall -std=c++11)
link_libraries(dl nsl m pthread rt)
add_definitions(-fpic -DRTI_UNIX -DRTI_LINUX -march=armv6 -mfpu=vfp -mfloat-abi=hard -mlong-calls -DRTI_STATIC)
include_directories(/home/janis/rti_connext_dds-5.3.1/include)
include_directories(/home/janis/rti_connext_dds-5.3.1/include/ndds)
include_directories(/home/janis/rti_connext_dds-5.3.1/include/ndds/hpp)
include_directories(rtidds)
link_libraries(nddscpp2z nddscz nddscorez )

link_directories(/home/janis/rti_connext_dds-5.3.1/lib/armv6vfphLinux3.xgcc4.7.2)


add_executable(hello_publisher 

rtidds/Pose2D.cxx
rtidds/Pose2D.hpp
rtidds/Pose2D_publisher.cxx
rtidds/Pose2DImpl.h
rtidds/Pose2DImplPlugin.h
rtidds/Pose2DImpl.cxx
rtidds/Pose2DImplPlugin.cxx


)

