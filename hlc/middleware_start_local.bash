#!/bin/bash

# Argument 1: Vehicle ID
# Argument 2: Node ID (Identifier of the middleware)
vehicle_id=$1
simulated_time=$2
middleware_id="middleware"

export IP_SELF="$(hostname -i)"
export IP_SELF="$(echo $IP_SELF)"
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

# Start screen for middleware; detach and start middleware
cd ./middleware/build

./middleware --node_id=${middleware_id} --vehicle_ids=${vehicle_id} --dds_domain=${DDS_DOMAIN} --simulated_time=${simulated_time} --dds_initial_peer=${DDS_INITIAL_PEER}