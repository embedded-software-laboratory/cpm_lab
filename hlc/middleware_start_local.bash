#!/bin/bash

# Argument 1: Vehicle ID
# Argument 2: Node ID (Identifier of the middleware)
vehicle_id=$1
simulated_time=$2
middleware_id="middleware"

# Start screen for middleware; detach and start middleware
cd ./middleware/build

# Put this into external file / find out if it is really necessary
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib64/:/tmp/cpm_base/cpm_lib/build/:/opt/rti_connext_dds-5.3.1/lib/x64Linux3gcc5.4.0
export PATH=$PATH:$HOME/bin:$HOME/.local/bin:/opt/rti_connext_dds-5.3.1:/opt/rti_connext_dds-5.3.1/bin:/opt/rti_connext_dds-5.3.1/lib/x64Linux3gcc5.4.0:/opt/rti_connext_dds-5.3.1/bin:/opt/raspbian-toolchain-gcc-4.7.2-linux64/bin
export NDDSHOME=/opt/rti_connext_dds-5.3.1
export RASPBIAN_TOOLCHAIN=/opt/raspbian-toolchain-gcc-4.7.2-linux64
export RTI_LICENSE_FILE=/opt/rti_connext_dds-5.3.1/rti_license.dat

export DDS_INITIAL_PEER=rtps@udpv4://192.168.1.249:25598

echo $middleware_id

./middleware --node_id=${middleware_id} --vehicle_ids=${vehicle_id} --dds_domain=3 --simulated_time=${simulated_time} --dds_initial_peer=${DDS_INITIAL_PEER}