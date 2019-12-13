#!/bin/bash


if [ ! -d "idl_compiled" ]; then
    mkdir idl_compiled
    find ./idl/ -type f | xargs -n 1 rtiddsgen -replace -legacyPlugin -language C++11 -d ./idl_compiled/ -I ../../../cpm_base/dds_idl/
fi

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