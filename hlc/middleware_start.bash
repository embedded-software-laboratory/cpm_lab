#!/bin/bash

# Argument 1: Vehicle ID
# Argument 2: Node ID (Identifier of the middleware)
vehicle_id=$1
simulated_time=$2

# Start screen for middleware; detach and start middleware
cd /tmp/hlc/middleware

./middleware --vehicle_ids=${vehicle_id} --dds_domain=3 --simulated_time=${simulated_time} #--dds_initial_peer=${DDS_INITIAL_PEER}