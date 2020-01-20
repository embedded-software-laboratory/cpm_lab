#!/bin/bash

mkdir -p build

# Copy local communication QoS, use correct IP
IP_SELF="$(hostname -I)"
sed -e "s/TEMPLATE_IP/${IP_SELF}/g" \
<./QOS_LOCAL_COMMUNICATION.xml.template \
>./build/QOS_LOCAL_COMMUNICATION.xml

cd build
cmake .. 
make -j8 && ./unittest
cd ..