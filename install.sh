#!/bin/bash
# This scripts automates the mandatory steps in order to compile the cpm
# software suit (/software/build_all.bash). As it installs build tools it needs
# super user privileges (e.g. sudo)
#
# This script will download various files and therefor creates its own
# directory 'tmp' relative to this scripts ablsoute path. It will also link it
# to ~/dev for easier accessablity (if ~/dev is not already present).
#
# This script is compatible with Debian and RedHat based distirbutions but has
# been tested specifically with Ubuntu 18.04.3 LTS and Fedora 31.
#
# NOTE: This script requires user input in step
# - '3.2 Installation'
#  -- to click mannually through the RTI Connext DDS 6.0.0 GUI installer
#  -- to provide an absolute path to an RTI license file
# - '3.3 Environment Setup' to enter a unique DDS Domain
# - Default Arguments may be passed via commandline ./setup_cpm_build_environment.sh path_to_rti_license domain_id

# This causes the bash script to return non-zero exit code as soon a command fails
set -e
# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command failed with exit code $?."' EXIT

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

### 0. Preconditioning #########################################################



## 0.1 Check for Super User Privileges and Handle User Rights
# ref: https://askubuntu.com/a/30157/8698
if ! [ $(id -u) = 0 ]; then
   echo "This script needs super user privileges. Try to run it again with sudo." >&2
   exit 1
fi
if [ $SUDO_USER ]; then
    real_user=$SUDO_USER
else
    real_user=$(whoami)
fi
RU_HOME=$( getent passwd $real_user | cut -d: -f6 )

## 0.2 Determine OS & Set Commands Accordinaly
if [[ ! -z $(which yum) ]]; then
    PM="yum"
    if [[ ! -z $(which dnf) ]]; then
        PM="dnf"
    fi
    echo "You aren't using Ubuntu. Watch out for further compatibility issues!"
    UPDATE="--refresh update -y"
    BUILD_ESSENTIALS="install gcc g++ glibc-devel libnsl2-devel make -y"
    # BUILD_ESSENTIALS="groupinstall \"Development Tools\" \"Development Libraries\" -y && dnf install libnsl2-devel g++ -y"
    # TODO Update if support for other OS is desired
    BUILD_TOOLS="install ip expect apache2 git tmux openssh-client openssh-server cmake gtkmm30-devel sshpass ntp -y"
    OPENJDK="install java-11-openjdk-devel -y"
    PYLON_URL="https://www.baslerweb.com/fp-1523350799/media/downloads/software/pylon_software/pylon-5.0.12.11829-x86_64.tar.gz"
elif [[ ! -z $(which apt) ]]; then
    PM="apt"
    UPDATE="update && apt upgrade -y"
    BUILD_ESSENTIALS="install build-essential -y"
    BUILD_TOOLS="install iproute2 git tmux cmake libgtkmm-3.0-dev libxml++2.6-dev ntp jstest-gtk openssh-client openssh-server sshpass -y"
    DEP_NO_SIM="install apache2 libgstreamer1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y"
    OPENJDK="install openjdk-11-jdk -y"
    PYLON_URL="https://www.baslerweb.com/fp-1523350893/media/downloads/software/pylon_software/pylon_5.0.12.11829-deb0_amd64.deb"
else
    echo "Error: unsupported package manager, make sure your are using apt, dnf or yum"
    exit 1;
fi

## 0.3 Link folder to '~/dev'
if [ ! -d $RU_HOME/dev/software ]; then
    echo "Creating a link to expected installation location ${RU_HOME}/dev/software"
    sudo -u ${real_user} mkdir -p $RU_HOME/dev
    sudo -u ${real_user} ln -s $PWD $RU_HOME/dev/software
elif [ "${RU_HOME}/dev/software" == "${PWD}" ]; then
    : # We are in the expected folder
else
    # TODO: Installation without relying on absolute location
    echo "WARNING: Installing at another location than /home/user/dev/software is currently not supported. Prepare for compatibility issues."
fi

### 0.4 Parse Command Line Arguments
CI=0
DOMAIN_ID="NONE"
LICENSE_PATH=0
SIMULATION=0
while [[ $# -gt 0 ]] && [[ "$1" == "--"* ]] ;
do
    opt="$1";
    shift;              #expose next argument
    case "$opt" in
        "--" ) break 2;;
        "--ci" )
           CI="1";;
        "--domain_id="* )
           DOMAIN_ID="${opt#*=}";;
        "--license_path="* )
           LICENSE_PATH="${opt#*=}";;
        "--simulation" )
           SIMULATION="1";;     #set to some default value
        "--rti_installer_automation_path="* )
           RTI_INSTALLER_AUTOMATION_PATH="${opt#*=}";;
        *) echo >&2 "Invalid option: $@"; exit 1;;
   esac
done

if [ "$LICENSE_PATH" == 0 ]; then
    read -p 'Ask your supervisor for a copy of the RTI license or get into contact with RTI and enter its absolute path (e.g. /home/max/rti_license.dat): ' LICENSE_PATH
    #check, if a license path was entered
    while [ -z "$LICENSE_PATH" ]; do
          echo "No license path was entered, please try again"
          read LICENSE_PATH
    done

    if [ -f "$LICENSE_PATH" ]; then
        echo "found $LICENSE_PATH"
    else
        echo "License file does not exist: $LICENSE_PATH"
        exit 1
    fi
fi

if [ "$DOMAIN_ID" == "NONE" ]; then
    read -p 'Please provide a integer Domain ID: ' DOMAIN_ID
    while [ -z "$DOMAIN_ID" ]; do
          echo "No domain id was specified"
          read DOMAIN_ID
    done

    if ! [[ "$DOMAIN_ID" =~ ^[0-9]+$ ]]
    then
        echo "please provide integer number for domain ID"
        exit 1
    fi
fi

echo "CI =" $CI
echo "Domain ID =" $DOMAIN_ID
echo "License Path =" $LICENSE_PATH
echo "Simulation =" $SIMULATION


### 0.5 Create folders for nuc and raspberry packages
if [[ $SIMULATION == 0 ]]; then
    sudo mkdir -p "/var/www/html/nuc"
    sudo chmod a+rwx "/var/www/html/nuc"
    sudo mkdir -p "/var/www/html/raspberry"
    sudo chmod a+rwx "/var/www/html/raspberry"
fi

### 0.6 Create temporary folder
sudo -u ${real_user} mkdir tmp

### 1. Ubuntu & Packages #######################################################
eval "${PM}" "${UPDATE}"
eval "${PM}" "${BUILD_ESSENTIALS}"
eval "${PM}" "${BUILD_TOOLS}"
if [ $SIMULATION == 0 ]; then
    eval "${PM}" "${DEP_NO_SIM}"
fi




### 2. Joystick / Gamepad ######################################################
#With a Joystick or a Gamepad you can drive vehicles manually in the Lab Control Center (LCC)
if [[ ! -z $(which yum) ]] || [[ ! -z $(which dnf) ]]; then
    cd tmp
    eval "${PM}" install libsigc++-devel gtkmm24-devel -y
    sudo -u $real_user git clone https://gitlab.com/jstest-gtk/jstest-gtk.git
    cd ./jstest-gtk/
    # checkout commit from 25 Aug, 2016 to match what is present in Ubuntu 18.04.3 LTS
    # TODO consider updating jstest-gtk because more recent versions don't require 
    # gtkmm24-devel anymore but are based on gtkmm30-devel like LCC.
    sudo -u $real_user git checkout c10e47cfa8d13516ce5234738857e796138aa3bd 
    sudo -u $real_user mkdir ./build
    cd ./build
    sudo -u $real_user cmake ..
    sudo -u $real_user make
fi


### 3. RTI DDS #################################################################
# RTI DDS is used for real-time communication between programs and devices. It
# implements a publish-subscribe pattern and serialization/deserialization for
# the messages.
# https://cpm.embedded.rwth-aachen.de/doc/display/CLD/RTI+DDS

## 3.1 Downloads
cd $DIR/tmp
sudo -u $real_user wget https://s3.amazonaws.com/RTI/Bundles/6.0.0/Evaluation/rti_connext_dds_secure-6.0.0-eval-x64Linux4gcc7.3.0.tar.gz
sudo -u $real_user tar xvzf ./rti_connext_dds_secure-6.0.0-eval-x64Linux4gcc7.3.0.tar.gz
if [ $SIMULATION == 0 ]; then
    sudo -u $real_user wget https://s3.amazonaws.com/RTI/Community/ports/toolchains/raspbian-toolchain-gcc-4.7.2-linux64.tar.gz
    sudo -u $real_user tar xvzf ./raspbian-toolchain-gcc-4.7.2-linux64.tar.gz
    cp -R raspbian-toolchain-gcc-4.7.2-linux64 /opt
    sudo -u $real_user wget https://community.rti.com/static/downloads/connext-dds/6.0.0/rti_connext_dds-6.0.0-core-target-armv6vfphLinux3.xgcc4.7.2.rtipkg
fi

## 3.2 Installation
mkdir /opt/rti_connext_dds-6.0.0
if [ $CI == 1 ]; then
    ${RTI_INSTALLER_AUTOMATION_PATH}
else
    echo "Unattended mode is not supported in the evaluation bundle thus you have to manually click through (click Forward, accecpt the license agreement and keep clicking Forward until you can click Finsih at the very last page)."
    ./rti_connext_dds-6.0.0-eval-x64Linux4gcc7.3.0.run --prefix /opt/rti_connext_dds-6.0.0
fi
cp "$LICENSE_PATH" /opt/rti_connext_dds-6.0.0/rti_license.dat

## 3.3 Environment Setup
echo "/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0" > /etc/ld.so.conf.d/rti_connext_dds.conf
ldconfig

# Select a unique DDS domain! To avoid interference from other users in the same
# network, you need to set a DDS domain ID that is different from everyone in
# the network. The domain ID is assumed to be in the environment variable
# DDS_DOMAIN.

echo "export DDS_DOMAIN=""${DOMAIN_ID}" > /etc/profile.d/rti_connext_dds.sh
echo "export PATH=\$PATH:/opt/rti_connext_dds-6.0.0/bin" >> /etc/profile.d/rti_connext_dds.sh
echo "export PATH=\$PATH:/opt/raspbian-toolchain-gcc-4.7.2-linux64/bin" >> /etc/profile.d/rti_connext_dds.sh
echo "export NDDSHOME=/opt/rti_connext_dds-6.0.0" >> /etc/profile.d/rti_connext_dds.sh
echo "export RASPBIAN_TOOLCHAIN=/opt/raspbian-toolchain-gcc-4.7.2-linux64" >> /etc/profile.d/rti_connext_dds.sh
echo "export RTI_LICENSE_FILE=/opt/rti_connext_dds-6.0.0/rti_license.dat" >> /etc/profile.d/rti_connext_dds.sh
echo "export RPIPWD=cpmcpmcpm" >> /etc/profile.d/rti_connext_dds.sh
# Reboot or source to apply the changes made to the environment variables.
source /etc/profile.d/rti_connext_dds.sh

## 3.4 Install RTI ARM libraries
# only needed in real lab mode
if [ $SIMULATION == 0 ]; then
    yes | /opt/rti_connext_dds-6.0.0/bin/rtipkginstall rti_connext_dds-6.0.0-core-target-armv6vfphLinux3.xgcc4.7.2.rtipkg
fi

### 4. Indoor Positioning System (Setup) #######################################
# The Indoor Positioning System depends on the camera software Basler Pylon and
# on OpenCV 4.0.0.
# https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Indoor+Positioning+System

if [ $SIMULATION == 0 ]
then
    ## 4.1 OpenCV 4.0.0
    eval "${PM}" "${OPENJDK}"
    cd /tmp
    sudo -u $real_user git clone https://github.com/opencv/opencv.git
    cd ./opencv
    sudo -u $real_user git checkout 4.0.0
    sudo -u $real_user mkdir ./build
    cd ./build
    sudo -u $real_user cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/opt/opencv400 ..
    N=$(nproc)
    sudo -u $real_user make -j$N
    make install
    cd "$DIR"
    if [ ! -d "/opt/opencv400/lib" ]; then
        ln -s /opt/opencv400/lib64 /opt/opencv400/lib
    fi
    rm -rf /tmp/opencv

    ## 4.2 Basler Pylon 5
    cd "$DIR/tmp"
    sudo -u $real_user wget "${PYLON_URL}"
    if [[ ! -z $(which yum) ]] || [[ ! -z $(which dnf) ]]; then
        sudo -u $real_user tar xvzf ./pylon*.tar.gz
        cd ./pylon*x86_64
        tar -C /opt -xzf pylonSDK*.tar.gz
        yes | ./setup-usb.sh
    elif [[ ! -z $(which apt) ]]; then
        dpkg -i pylon*.deb
    fi
fi

rm -rf "${DIR}/tmp"

### 5. Inform user about success and next steps ################################
echo "Success! Ready to build the cpm software suit."
echo "Reboot your PC or execute 'source /etc/profile.d/rti_connext_dds.sh'"
echo "Then: './build_all.bash' or './build_all.bash --simulation'"

exit 0