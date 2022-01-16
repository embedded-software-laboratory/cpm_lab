#!/bin/bash
# exit when any command fails
set -e

mkdir -p build

# Copy local communication QoS, use correct IP
IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
sed -e "s/TEMPLATE_IP/${IP_SELF}/g" \
<./QOS_LOCAL_COMMUNICATION.xml.template \
>./build/QOS_LOCAL_COMMUNICATION.xml

./rtigen.bash
cd build
cmake .. 
make -j$(nproc)
cd ..
