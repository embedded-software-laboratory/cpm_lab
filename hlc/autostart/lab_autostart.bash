#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib64/:/tmp/software/cpm_library_package/:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0
export PATH=$PATH:$HOME/bin:$HOME/.local/bin:/opt/rti_connext_dds-6.0.0:/opt/rti_connext_dds-6.0.0/bin:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0:/opt/rti_connext_dds-6.0.0/bin:/opt/raspbian-toolchain-gcc-4.7.2-linux64/bin
export NDDSHOME=/opt/rti_connext_dds-6.0.0
export RASPBIAN_TOOLCHAIN=/opt/raspbian-toolchain-gcc-4.7.2-linux64
export RTI_LICENSE_FILE=/opt/rti_connext_dds-6.0.0/rti_license.dat

export DDS_INITIAL_PEER=rtps@udpv4://192.168.1.249:25598

# Ping to make sure that an internet connection is available
# Write to /dev/null to suppress output
while ! ping -c 1 -w 1 192.168.1.249 &>/dev/null
do
	sleep 1
done

# Get relevant HLC software - cpm library, middleware and autostart program that signals that the HLC is ready
cd /tmp
rm -rf ./software
mkdir software
cd ./software
wget http://192.168.1.249/nuc/middleware_package.tar.gz
tar -xzvf middleware_package.tar.gz
wget http://192.168.1.249/nuc/cpm_library_package.tar.gz
tar -xzvf cpm_library_package.tar.gz
wget http://192.168.1.249/nuc/autostart_package.tar.gz
tar -xzvf autostart_package.tar.gz
chmod -R a+rwx ../software # Make folder accessible to guest user

# NEW: Now mimic the folder structure the scripts expect (for the guest user)
cd /home/guest/
rm -rf ./dev
mkdir -p dev/cpm_base/cpm_lib
cd ./dev/cpm_base/cpm_lib
mkdir build
cp /tmp/software/cpm_library_package/libcpm.so ./build
cp -R /tmp/software/cpm_library_package/dds_idl_matlab ./

cd /home/guest/dev
mkdir -p software/hlc/middleware/build
cp /tmp/software/middleware_package/middleware ./software/hlc/middleware/build
# Set correct IP in local communication script
my_ip=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
cp -rf /tmp/software/middleware_package/QOS_LOCAL_COMMUNICATION.xml.template ./software/hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml
sed -i -e "s/TEMPLATE_IP/${my_ip}/g" ./QOS_LOCAL_COMMUNICATION.xml

# Default domain is 21, just like the vehicle default domain (-> domain for real lab tests)
/tmp/software/autostart_package/autostart --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER