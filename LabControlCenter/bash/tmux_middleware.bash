#!/bin/bash
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
cd ./middleware/build

echo $middleware_id

./middleware {MIDDLEWARE_ARGS}