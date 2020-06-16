#!/bin/bash

# Argument 1: HLC script name
# Argument 2: Vehicle ID
script_path=$1
script_name=$2
vehicle_id=$3
simulated_time=$4

# Get the current script and middleware versions from the Apache server
cd /tmp
rm nuc_package.tar.gz
wget http://192.168.1.249/nuc/nuc_package.tar.gz
tar -xzvf nuc_package.tar.gz

# bash "./high_level_controller/start.bash ${script_path} ${script_name} ${vehicle_id} ${simulated_time}"
tmux new-session -d -s "start_script" "cd /tmp/software/high_level_controller/;bash start.bash ${script_path} ${script_name} ${vehicle_id} ${simulated_time}"