#!/bin/bash


export IP_SELF="$(hostname -i)"

echo IP_SELF
echo $IP_SELF

export VEHICLE_ID="$(cat vehicle_id_map/$IP_SELF)"

echo VEHICLE_ID
echo $VEHICLE_ID

LD_LIBRARY_PATH=/tmp/package chrt -r 99 ./vehicle_rpi_firmware --simulated_time=false --vehicle_id=$VEHICLE_ID >stdout_$VEHICLE_ID.txt 2>stderr_$VEHICLE_ID.txt