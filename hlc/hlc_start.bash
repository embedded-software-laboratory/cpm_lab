#!/bin/bash
script_path=$1
script_name=$2
vehicle_id=$3

#Load environment variables, like RTI location, library location, Matlab location...
. ./environment_variables.bash

/opt/MATLAB/R2019a/bin/matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -r "cd '/tmp/software/hlc/${script_path}'; ${script_name}(1, ${vehicle_id})" #1 is the local comm. domain ID, cannot be changed currently (is probably also not necessary)