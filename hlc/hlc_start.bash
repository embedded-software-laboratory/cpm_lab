#!/bin/bash
script_path=$1
script_name=$2
vehicle_id=$3

# Put this into external file / find out if it is really necessary
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib64/:/tmp/cpm_base/cpm_lib/build/:/opt/rti_connext_dds-5.3.1/lib/x64Linux3gcc5.4.0
export PATH=$PATH:$HOME/bin:$HOME/.local/bin:/opt/rti_connext_dds-5.3.1:/opt/rti_connext_dds-5.3.1/bin:/opt/rti_connext_dds-5.3.1/lib/x64Linux3gcc5.4.0:/opt/rti_connext_dds-5.3.1/bin:/opt/raspbian-toolchain-gcc-4.7.2-linux64/bin
export NDDSHOME=/opt/rti_connext_dds-5.3.1
export RASPBIAN_TOOLCHAIN=/opt/raspbian-toolchain-gcc-4.7.2-linux64
export RTI_LICENSE_FILE=/opt/rti_connext_dds-5.3.1/rti_license.dat

/opt/MATLAB/R2018b/bin/matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -r "cd '/tmp/software/hlc/${script_path}'; ${script_name}(1, '${vehicle_id}')" #1 is the local comm. domain ID, cannot be changed currently (is probably also not necessary)