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

# Make middleware and unittest
cmake .. 
make -j8

# Publish middleware package via http/apache for the HLCs to download
cd /tmp
mkdir middleware_package
cp ~/dev/software/hlc/middleware/build/middleware ./middleware_package
cp ~/dev/software/hlc/middleware/build/libadditional_idl.so ./middleware_package
cp ~/dev/software/hlc/middleware/QOS_LOCAL_COMMUNICATION.xml.template ./middleware_package
cp ~/dev/software/hlc/middleware/idl/VehicleStateList.idl ./middleware_package
tar -czvf middleware_package.tar.gz middleware_package
rm -f /var/www/html/nuc/middleware_package.tar.gz
cp ./middleware_package.tar.gz /var/www/html/nuc

# Perform unittest
cd ~/dev/software/hlc/middleware/build 
./unittest