#!bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib64/:./cpm:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0
export PATH=$PATH:$HOME/bin:$HOME/.local/bin:/opt/rti_connext_dds-6.0.0:/opt/rti_connext_dds-6.0.0/bin:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0:/opt/rti_connext_dds-6.0.0/bin:/opt/raspbian-toolchain-gcc-4.7.2-linux64/bin
export NDDSHOME=/opt/rti_connext_dds-6.0.0
export RASPBIAN_TOOLCHAIN=/opt/raspbian-toolchain-gcc-4.7.2-linux64
export RTI_LICENSE_FILE=/opt/rti_connext_dds-6.0.0/rti_license.dat

export DDS_INITIAL_PEER=rtps@udpv4://192.168.1.249:25598

exit_script() {
    tmux kill-session -t "rticlouddiscoveryservice"
}

trap exit_script SIGINT SIGTERM

# Start local software (WARNING: domain hardcoded right now)
# tmux new-session -d -s "rticlouddiscoveryservice" "rticlouddiscoveryservice -transport 25598  >stdout_rticlouddiscoveryservice.txt 2>stderr_rticlouddiscoveryservice.txt"

# Default domain is 21, just like the vehicle default domain (-> domain for real lab tests)
./build/dummy_program_check --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER