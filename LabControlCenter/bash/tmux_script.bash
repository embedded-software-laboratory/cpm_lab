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

#Omit /home/username and script name
PATH_TO_SCRIPT="${SCRIPT_PATH#*home/}"
PATH_TO_SCRIPT="~/${PATH_TO_SCRIPT#*/}"
PATH_TO_SCRIPT="${PATH_TO_SCRIPT%/*}" #Get string before last / (omit name of script)

#Load environment variables, like RTI location, library location, Matlab location...
. ./environment_variables.bash

# Copy local communication XML
eval "cp -rf ~/dev/software/hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml  ${PATH_TO_SCRIPT}"

cd /tmp/
echo "Path to script: ${PATH_TO_SCRIPT} and script name: ${SCRIPT_NAME}" > script_path.log

#Start either a Matlab script using Matlab or a C++ script
if [[ ${SCRIPT_NAME} =~ ".m" ]]
then
    SCRIPT_NAME="${SCRIPT_NAME%%.*}" #remove .m
    /opt/MATLAB/R2019a/bin/matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -r "cd ${PATH_TO_SCRIPT}; ${SCRIPT_NAME}(${SCRIPT_ARGS})"
else
    eval "${PATH_TO_SCRIPT}/${SCRIPT_NAME} ${SCRIPT_ARGS} &> script.log"
fi