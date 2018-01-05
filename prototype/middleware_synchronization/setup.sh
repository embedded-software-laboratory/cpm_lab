#!/bin/bash

clear
source /opt/ros/kinetic/setup.bash
catkin_make $@
if [ $? -eq 0 ]; then
    source devel/setup.bash
    doxygen doc/Doxyfile
    rosrun ips ips_unittest
    rosrun cpm_tools cpm_tools_unittest
fi

for i in doc/manuals/*.md; do
    [ -f "$i" ] || break
    pandoc --css style.css $i >$i.html
done
