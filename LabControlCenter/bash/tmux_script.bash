#!/bin/bash
#Get command line arguments
for i in "$@"
do
case $i in
    --script_path=*)
    SCRIPT_PATH="${i#*=}"
    shift # past argument=value
    ;;
    --script_arguments=*)
    SCRIPT_ARGS="${i#*=}"
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

if []
then
    
fi

#Load environment variables, like RTI location, library location, Matlab location...
. ./environment_variables.bash

/opt/MATLAB/R2019a/bin/matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -r "cd '/tmp/software/hlc/${script_path}'; ${script_name}(1, '${vehicle_id}')" #1 is the local comm. domain ID, cannot be changed currently (is probably also not necessary)