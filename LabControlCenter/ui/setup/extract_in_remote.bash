#!/bin/bash

cd /tmp/software/

# Set correct IP in local communication script
my_ip=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
/bin/cp -rf ./hlc/middleware/QOS_LOCAL_COMMUNICATION.xml.template ./hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml
sed -i -e "s/TEMPLATE_IP/${my_ip}/g" ./hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml
/bin/cp -rf ./hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml ./hlc/$script_path/

# Copy cpm lib file
/bin/cp ./cpm_base/cpm_lib/build/libcpm.so ./hlc/middleware/build/