# cpm_base

Common interfaces and libraries, used by all CPM Lab projects.


# Setup Notes


## RTI Connext

Download & install RTI Connext from 

https://www.rti.com/free-trial/dds-files
https://s3.amazonaws.com/RTI/Bundles/5.3.1/Evaluation/rti_connext_dds_secure-5.3.1-eval-x64Linux3gcc5.4.0.tar.gz


## Raspberry PI with RTI Connext

https://community.rti.com/content/forum-topic/howto-run-rti-connext-dds-raspberry-pi

wget https://s3.amazonaws.com/RTI/Community/ports/toolchains/raspbian-toolchain-gcc-4.7.2-linux64.tar.gz
tar xvzf raspbian-toolchain-gcc-4.7.2-linux64.tar.gz

Raspberry PI RTI Libraries

https://community.rti.com/downloads/rti-connext-dds-raspberry-pi

## Bash RC

    export PATH=$PATH:$HOME/rti_connext_dds-5.3.1/bin
    export PATH=$PATH:$HOME/toolchains/raspbian-toolchain-gcc-4.7.2-linux64/bin
    export NDDSHOME=$HOME/rti_connext_dds-5.3.1
    export RASPBIAN_TOOLCHAIN=$HOME/raspbian-toolchain-gcc-4.7.2-linux64
    export RTI_LICENSE_FILE=$HOME/rti_workspace/rti_license.dat
