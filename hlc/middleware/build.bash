#!/bin/bash

mkdir -p build

# Copy local communication QoS, use correct IP
IP_SELF="$(hostname -I)"
sed -e "s/TEMPLATE_IP/${IP_SELF}/g" \
<./QOS_LOCAL_COMMUNICATION.xml.template \
>./build/QOS_LOCAL_COMMUNICATION.xml

cd build
cmake .. 
make -j8

# Publish middleware package via http/apache for the HLCs to download
cd /tmp
mkdir middleware_package
cp ~/dev/software/hlc/middleware/build/middleware ./middleware_package
cp ~/dev/software/hlc/middleware/QOS_LOCAL_COMMUNICATION.xml.template ./middleware_package
tar -czvf middleware_package.tar.gz middleware_package
rm -f /var/www/html/nuc/middleware_package.tar.gz
cp ./middleware_package.tar.gz /var/www/html/nuc

# Perform unittest
cd ~/dev/software/hlc/middleware/build 
./unittest