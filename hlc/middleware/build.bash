#!/bin/bash


if [ ! -d "idl_compiled" ]; then
    mkdir idl_compiled
    find ./idl/ -type f | xargs -n 1 rtiddsgen -replace -legacyPlugin -language C++11 -d ./idl_compiled/ -I ../../../cpm_base/dds_idl/
fi

clear

mkdir build
cd build

# Copy local communication QoS, use correct IP
my_ip=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
sed -e "s/TEMPLATE_IP/${my_ip}/g" \
<../QOS_LOCAL_COMMUNICATION.xml.template \
>./QOS_LOCAL_COMMUNICATION.xml

cmake .. 
make -j8 && ./unittest
cd ..