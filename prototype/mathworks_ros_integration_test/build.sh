#!/bin/bash

clear
source /opt/ros/kinetic/setup.bash
roscore &


cd('src/controller_in_simulink/'); load_system('controller_in_simulink'); rtwbuild('controller_in_simulink'); exit

load_system('controller_in_simulink')