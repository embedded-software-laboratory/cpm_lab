

# Toolchain downloaded from
# https://s3.amazonaws.com/RTI/Community/ports/toolchains/raspbian-toolchain-gcc-4.7.2-linux64.tar.gz


# Raspberry PI RTI Libraries

https://community.rti.com/downloads/rti-connext-dds-raspberry-pi

# bashrc:

export PATH=$PATH:/home/janis/rti_connext_dds-5.3.1/bin
export PATH=/home/janis/toolchains/raspbian-toolchain-gcc-4.7.2-linux64/bin:$PATH
export RTI_LICENSE_FILE=/home/janis/rti_workspace/rti_license.dat
export RASPBIAN_TOOLCHAIN=/home/janis/toolchains/raspbian-toolchain-gcc-4.7.2-linux64
export NDDSHOME=/home/janis/rti_connext_dds-5.3.1