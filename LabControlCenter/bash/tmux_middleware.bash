#!/bin/bash
#Get command line arguments
for i in "$@"
do
case $i in
    --middleware_id=*)
    MIDDLEWARE_ID="${i#*=}"
    shift # past argument=value
    ;;
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

./middleware --node_id=${middleware_id} --vehicle_ids=${vehicle_id} --dds_domain=21 --simulated_time=${simulated_time} --dds_initial_peer=${DDS_INITIAL_PEER}