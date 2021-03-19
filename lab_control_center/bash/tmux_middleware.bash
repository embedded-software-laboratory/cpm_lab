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

# Get middlware build directory and environment variables directory relative to this script's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
RELATIVE_MIDDLEWARE_DIR="${DIR}../../middleware/build"
ABSOLUTE_MIDDLEWARE_DIR="$(realpath "${RELATIVE_MIDDLEWARE_DIR}")"
RELATIVE_LOG_DIR="${DIR}../../../lcc_script_logs"
ABSOLUTE_LOG_DIR="$(realpath "${RELATIVE_LOG_DIR}")"

#Load environment variables, like RTI location, library location, Matlab location...
cd $DIR
. ./environment_variables.bash

# Start screen for middleware; detach and start middleware
cd ${ABSOLUTE_MIDDLEWARE_DIR}

./middleware ${MIDDLEWARE_ARGS} &> ${ABSOLUTE_LOG_DIR}/middleware.log