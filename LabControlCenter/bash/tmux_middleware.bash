#!/bin/bash
# This file is used on the remote system (NUC)
# DESCRIPTION: Start the middleware. Load relevant environment variables beforehand.
#Get command line arguments
for i in "$@"
do
case $i in
    --middleware_arguments=*)
    MIDDLEWARE_ARGS="${i#*=}"
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

#Load environment variables, like RTI location, library location, Matlab location...
. ./environment_variables.bash

# Start screen for middleware; detach and start middleware
cd ~/dev/software/middleware/build

./middleware ${MIDDLEWARE_ARGS} &> ~/dev/lcc_script_logs/middleware.log