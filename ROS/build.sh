#!/bin/bash

source /opt/ros/kinetic/setup.bash
catkin_make $@
source devel/setup.bash
doxygen doc/Doxyfile
rosrun ips unittest