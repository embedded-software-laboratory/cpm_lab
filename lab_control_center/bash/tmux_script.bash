#!/bin/bash
# This file is used on the remote system (NUC)
# DESCRIPTION: Start the script - either Matlab or C++. Load relevant environment variables beforehand.
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

#Omit /home/username and script name
PATH_TO_SCRIPT="${SCRIPT_PATH#*home/}"
PATH_TO_SCRIPT="~/${PATH_TO_SCRIPT#*/}"
PATH_TO_SCRIPT="${PATH_TO_SCRIPT%/*}" #Get string before last / (omit name of script)

#Load environment variables, like RTI location, library location, Matlab location...
. ./environment_variables.bash

# Copy local communication XML - eval must be used to evaluate the script, as it is given as a string and not as a bash command
eval "cp -rf ~/dev/software/middleware/build/QOS_LOCAL_COMMUNICATION.xml  ${PATH_TO_SCRIPT}"

cd /tmp/
#Create debug log file to check if the script path and name were extracted correctly
echo "Path to script: ${PATH_TO_SCRIPT} and script name: ${SCRIPT_NAME}" > ~/dev/lcc_script_logs/script_path.log

#Start either a Matlab script using Matlab or a C++ script
if [[ ${SCRIPT_NAME} =~ ".m" ]]
then
    #Evaluate the matlab script
    SCRIPT_NAME="${SCRIPT_NAME%%.*}" #remove .m
    /opt/MATLAB/R2020a/bin/matlab -logfile matlab.log -sd "${PATH_TO_SCRIPT}" -batch "${SCRIPT_NAME}(${SCRIPT_ARGS})"
else
    #Evaluate the C++ script
    eval "${PATH_TO_SCRIPT}/${SCRIPT_NAME} ${SCRIPT_ARGS} &> ~/dev/lcc_script_logs/script.log"
fi
