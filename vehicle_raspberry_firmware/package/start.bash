#!/bin/bash


export IP_SELF="$(hostname -I)"

echo IP_SELF
echo $IP_SELF

export VEHICLE_ID="$(hostname -I | tail -c 4)"
export VEHICLE_ID="$(echo $VEHICLE_ID)"


echo VEHICLE_ID
echo $VEHICLE_ID

wget http://192.168.1.249/raspberry/DDS_DOMAIN


LD_LIBRARY_PATH=/tmp/package chrt -r 99 ./vehicle_rpi_firmware --simulated_time=false --dds_domain=$(cat DDS_DOMAIN) --vehicle_id=$VEHICLE_ID >stdout_$VEHICLE_ID.txt 2>stderr_$VEHICLE_ID.txt
