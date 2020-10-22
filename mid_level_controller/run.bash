#!/bin/bash


export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

./build_x64_sim/vehicle_rpi_firmware --simulated_time=false --vehicle_id=$1 --pose=$2 --dds_domain=$DDS_DOMAIN --dds_initial_peer=$DDS_INITIAL_PEER >stdout_$1.txt 2>stderr_$1.txt
