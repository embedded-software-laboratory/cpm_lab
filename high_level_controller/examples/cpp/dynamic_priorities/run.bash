#!/bin/bash


export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

# only one vehicle id is to be specified as the hlc has to be run distributedly
./build/dynamic_priorities  --vehicle_ids=1 --hlc_mode=2
