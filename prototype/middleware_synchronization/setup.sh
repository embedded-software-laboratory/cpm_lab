#!/bin/bash

clear
source /opt/ros/kinetic/setup.bash
catkin_make $@
if [ $? -eq 0 ]; then
    source devel/setup.bash
    rosrun cpm_tools cpm_tools_unittest
fi