#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib64/:~/dev/software/cpm_lib/build:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0
export PATH=$PATH:$HOME/bin:$HOME/.local/bin:/opt/rti_connext_dds-6.0.0:/opt/rti_connext_dds-6.0.0/bin:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0:/opt/rti_connext_dds-6.0.0/bin:/opt/raspbian-toolchain-gcc-4.7.2-linux64/bin
export NDDSHOME=/opt/rti_connext_dds-6.0.0
export RASPBIAN_TOOLCHAIN=/opt/raspbian-toolchain-gcc-4.7.2-linux64
export RTI_LICENSE_FILE=/opt/rti_connext_dds-6.0.0/rti_license.dat

export DDS_INITIAL_PEER=rtps@udpv4://192.168.1.249:25598

# --------------------------------- WAIT FOR TIME SYNC --------------------------------------------------------
# Wait for clock sync before doing anything else, because starting any program before a clock sync would cause problems
# The clock sync is performed in rc.local of the sudo user
# We communicate via pipes, alternatively starting this script as another user would have been possible as well
# Check for existing pipe, wait until it has been created
nuc_ntp_pipe=/tmp/nuc_ntp_pipe
nuc_lab_pipe=/tmp/nuc_lab_pipe

while [[ ! (-p $nuc_ntp_pipe && -p $nuc_lab_pipe ) ]]; do
        sleep 1
done

# Read msg from pipe, which is sent after the time sync, then answer
# 1 -> time sync done, answer with 1 on other pipe
while read sy < $nuc_ntp_pipe; do
	if [[ $sy -eq 1 ]]; then
		break
	fi
	sleep 1
done
echo "1" > $nuc_lab_pipe
# ---------------------------------------------------------------------------------------------------------------

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

# Get the cpm library
out=$(wget http://192.168.1.249/nuc/cpm_library_package.tar.gz)
if [ $? -ne 0 ]; then
	#Behaviour in case the package is missing
	#Start some software here that sends this info to the LCC - the script blocks here (indefinitely)
	/home/guest/autostart/download_error_logger --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER
fi
tar -xzvf cpm_library_package.tar.gz

# The cpm library should already be available for download_error_logger (s.t. it can start in case it already exists and only the cpm_library was transferred this time)
# This might lead to errors if the cpm library gets updated, but the error_logger doesn't
cd /home/guest/
rm -rf ./dev
mkdir -p dev/software/cpm_lib
cd ./dev/software/cpm_lib
mkdir -p build
cp /tmp/software/cpm_library_package/libcpm.so ./build
cp -R /tmp/software/cpm_library_package/dds_idl_matlab ./

# Get the autostart software, and with it the error logger that is used in case a download failed
cd /tmp/software
out=$(wget http://192.168.1.249/nuc/autostart_package.tar.gz)
if [ $? -ne 0 ]; then
	#Behaviour in case the package is missing
	#Start some software here that sends this info to the LCC - the script blocks here (indefinitely)
	/home/guest/autostart/download_error_logger --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER
fi
tar -xzvf autostart_package.tar.gz

# Copy the autostart error logging software in the autostart folder, s.t. it can be started even if it is not available in a new package
cp /tmp/software/autostart_package/download_error_logger /home/guest/autostart

# Also replace the current lab_autostart bash file with the updated one; this will, of course, not be applied until the next restart
cp /tmp/software/autostart_package/lab_autostart.bash /home/guest/autostart

# Get the middleware & QoS
out=$(wget http://192.168.1.249/nuc/middleware_package.tar.gz)
if [ $? -ne 0 ]; then
	#Behaviour in case the package is missing
	#Start some software here that sends this info to the LCC - the script blocks here (indefinitely)
	/home/guest/autostart/download_error_logger --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER
fi
tar -xzvf middleware_package.tar.gz

# Put the middleware program & QoS in the correct folder
cd /home/guest/dev
mkdir -p software/middleware/build
cp /tmp/software/middleware_package/middleware ./software/middleware/build
# Set correct IP in local communication script
my_ip=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
cp -rf /tmp/software/middleware_package/QOS_LOCAL_COMMUNICATION.xml.template ./software/middleware/build/QOS_LOCAL_COMMUNICATION.xml
sed -i -e "s/TEMPLATE_IP/${my_ip}/g" ./software/middleware/build/QOS_LOCAL_COMMUNICATION.xml

# Do the same for cpm_libs QoS
cp /tmp/software/cpm_library_package/QOS_LOCAL_COMMUNICATION.xml.template ./software/cpm_lib/build/QOS_LOCAL_COMMUNICATION.xml
sed -i -e "s/TEMPLATE_IP/${my_ip}/g" ./software/cpm_lib/build/QOS_LOCAL_COMMUNICATION.xml

# Get Matlab init scripts
cd /tmp/software
out=$(wget http://192.168.1.249/nuc/matlab_package.tar.gz)
if [ $? -ne 0 ]; then
	#Behaviour in case the package is missing
	#Start some software here that sends this info to the LCC - the script blocks here (indefinitely)
	/home/guest/autostart/download_error_logger --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER
fi
tar -xzvf matlab_package.tar.gz
chmod -R a+rwx ../software # Make folder accessible to guest user

# Put the init scripts for Matlab in the correct folder
cd /home/guest/dev/software
mkdir high_level_controller
cd ./high_level_controller
mkdir examples
cd ./examples
mkdir matlab
cd ./matlab
cp /tmp/software/matlab_package/read_system_trigger.m ./
cp /tmp/software/matlab_package/init_script.m ./
cp /tmp/software/matlab_package/QOS_READY_TRIGGER.xml ./

cd ~

# Default domain is 21, just like the vehicle default domain (-> domain for real lab tests)
/tmp/software/autostart_package/autostart --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER  &> ~/autostart/log.log 
