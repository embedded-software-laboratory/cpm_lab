#!/bin/bash
script_path=$1
vehicle_id=$2

script_dir=$(dirname "$script_path")
script_name=$(basename --suffix=.m "$script_path")

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0

 # first parameter in .m function is the local comm. domain ID=1
matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -sd $script_dir -r "$script_name(1, ${vehicle_id})"