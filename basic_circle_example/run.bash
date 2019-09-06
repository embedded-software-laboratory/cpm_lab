#!/bin/bash


export IP_SELF="$(hostname -I)"
export IP_SELF="$(echo $IP_SELF)"
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

if [ $# -eq 0 ]
  then
    echo "Missing argument vehicle ID list (comma separated)"
else
	./build/basic_circle_example --dds_domain=$DDS_DOMAIN --dds_initial_peer=$DDS_INITIAL_PEER --vehicle_ids=$1
fi
