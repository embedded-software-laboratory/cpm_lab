#!/bin/bash


export IP_SELF="$(hostname -I)"
export IP_SELF="$(echo $IP_SELF)"
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

./build_x64_sim/vehicle_rpi_firmware --simulated_time=$3 --vehicle_id=$1 --dds_domain=$2 --dds_initial_peer=$DDS_INITIAL_PEER >stdout_$1.txt 2>stderr_$1.txt
