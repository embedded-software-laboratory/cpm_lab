#!/bin/bash

# Argument 1: Vehicle ID
# Argument 2: Node ID (Identifier of the middleware)
vehicle_ids=$1
simulated_time=$2
middleware_id=$(printf "middleware_${vehicle_ids}" )
echo $middleware_id

export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

# Start screen for middleware; detach and start middleware
cd ./build
./middleware --node_id=${middleware_id} --vehicle_ids=${vehicle_ids} --dds_domain=21 --simulated_time=${simulated_time} --dds_initial_peer=${DDS_INITIAL_PEER}