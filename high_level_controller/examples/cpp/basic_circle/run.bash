#!/bin/bash


export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

if [ $# -eq 0 ]
  then
    echo "Missing argument vehicle ID"
else
	./build/basic_circle --dds_domain=$DDS_DOMAIN --dds_initial_peer=$DDS_INITIAL_PEER --vehicle_id=$1
fi
