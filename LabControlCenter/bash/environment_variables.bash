# Put this into external file / find out if it is really necessary
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib64/:/tmp/software/cpm_library_package:/tmp/software/middleware_package:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0
export PATH=$PATH:$HOME/bin:$HOME/.local/bin:/opt/rti_connext_dds-6.0.0:/opt/rti_connext_dds-6.0.0/bin:/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0:/opt/rti_connext_dds-6.0.0/bin:/opt/raspbian-toolchain-gcc-4.7.2-linux64/bin
export NDDSHOME=/opt/rti_connext_dds-6.0.0
export RASPBIAN_TOOLCHAIN=/opt/raspbian-toolchain-gcc-4.7.2-linux64
export RTI_LICENSE_FILE=/opt/rti_connext_dds-6.0.0/rti_license.dat

# Replace this if you want the software to run on a PC that is not the Lab's main PC
# export IP_SELF="$(hostname -I)"
# export IP_SELF="$(echo $IP_SELF)"
# export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

export DDS_INITIAL_PEER=rtps@udpv4://192.168.1.249:25598