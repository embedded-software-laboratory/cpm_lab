#!/bin/bash

clear
kill $(jobs -p)
source /opt/ros/kinetic/setup.bash
roscore &
/opt/MATLAB/R2017b/bin/matlab -nodisplay -nodesktop -r "cd('src/controller_in_simulink/'); load_system('controller'); rtwbuild('controller'); exit"
rm -rf src/controller_in_simulink/controller_ert_rtw/
src/controller_in_simulink/build_ros_model.sh src/controller_in_simulink/controller.tgz $(pwd)
catkin_make
. devel/setup.bash
kill $(jobs -p)
sleep 3

roscore &
rosrun controller controller_node &
rosrun reference_position_in_cpp reference_position_in_cpp &
rqt_plot position reference &
/opt/MATLAB/R2017b/bin/matlab -nodisplay -nodesktop -r "cd('src/simulation_in_matlab/'); main;" </dev/null >/dev/null &
