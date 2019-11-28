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

#Extract script name and script path from SCRIPT_PATH
SCRIPT_NAME="${SCRIPT_PATH##*/}" #Get string after last /
PATH_TO_SCRIPT="${SCRIPT_PATH%/*}" #Get string before last /

#Load environment variables, like RTI location, library location, Matlab location...
. ./environment_variables.bash

#Start either a Matlab script using Matlab or a C++ script
if [[ ${SCRIPT_NAME} =~ ".m" ]]
then
    SCRIPT_NAME="${SCRIPT_NAME%%.*}" #remove .m
    /opt/MATLAB/R2019a/bin/matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -r "cd '/tmp/software${PATH_TO_SCRIPT}'; ${SCRIPT_NAME}(${SCRIPT_ARGS})"
else
    .${SCRIPT_PATH} ${SCRIPT_ARGS}
fi