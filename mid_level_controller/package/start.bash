#!/bin/bash


export IP_SELF="$(hostname -I)"

echo IP_SELF
echo $IP_SELF

# Copy local communication QoS, use correct IP
sed -e "s/TEMPLATE_IP/${IP_SELF}/g" \
<$DIR/QOS_LOCAL_COMMUNICATION.xml.template \
>$DIR/build/QOS_LOCAL_COMMUNICATION.xml

export VEHICLE_ID="$(hostname -I | tail -c 4)"
export VEHICLE_ID="$(echo $VEHICLE_ID)"


echo VEHICLE_ID
echo $VEHICLE_ID

wget http://192.168.1.249/raspberry/DDS_DOMAIN
wget http://192.168.1.249/raspberry/DDS_INITIAL_PEER


LD_LIBRARY_PATH=/tmp/package chrt -r 99 ./vehicle_rpi_firmware --simulated_time=false --dds_domain=$(cat DDS_DOMAIN) --dds_initial_peer=$(cat DDS_INITIAL_PEER) --vehicle_id=$VEHICLE_ID >stdout_$VEHICLE_ID.txt 2>stderr_$VEHICLE_ID.txt
