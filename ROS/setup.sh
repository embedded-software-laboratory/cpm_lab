#!/bin/bash

clear
source /opt/ros/kinetic/setup.bash
catkin_make $@
if [ $? -eq 0 ]; then
    source devel/setup.bash
    doxygen doc/Doxyfile
    rosrun ips unittest
fi

for i in doc/manuals/*.md; do
    [ -f "$i" ] || break
    pandoc --css style.css $i >$i.html
done
