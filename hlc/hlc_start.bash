#!/bin/bash
script_path=$1
script_name=$2
vehicle_id=$3

/opt/MATLAB/R2018b/bin/matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -r "cd 'tmp/hlc/${script_path}'; ${script_name}(3, 1, '${vehicle_id}')" #1 is the local comm. domain ID, cannot be changed currently