#!/bin/bash
script_path=$1
vehicle_id=$2

script_dir=$(dirname "$script_path")
script_name=$(basename --suffix=.m "$script_path")

 # first parameter in .m function is the local comm. domain ID=1
/opt/MATLAB/R2019a/bin/matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -sd $script_dir -r "$script_name(1, ${vehicle_id})"