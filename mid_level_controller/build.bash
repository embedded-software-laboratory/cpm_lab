#!/bin/bash
# exit when any command fails
set -e

mkdir -p build_arm
mkdir -p build_arm_sim
mkdir -p build_x64_sim

# Build for simulation on desktop
pushd build_x64_sim
cmake .. -DBUILD_ARM=OFF -DBUILD_SIMULATION=ON
make -j$(nproc)
popd

if [ -z $SIMULATION ]; then
    # Build for simulation on Raspberry
    pushd build_arm_sim
    cmake .. -DBUILD_ARM=ON -DBUILD_SIMULATION=ON -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
    make -j$(nproc)
    cp -R ../package/ .
    cp vehicle_rpi_firmware package
    cp ../../cpm_lib/build_arm/libcpm.so package
    tar -czf package.tar.gz package
    popd
    
    
    # Build for normal operation on Raspberry
    pushd build_arm
    cmake .. -DBUILD_ARM=ON -DBUILD_SIMULATION=OFF -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
    make -j$(nproc)
    cp -R ../package/ .
    cp vehicle_rpi_firmware package
    cp ../../cpm_lib/build_arm/libcpm.so package
    cp ../../cpm_lib/QOS_LOCAL_COMMUNICATION.xml.template package
    tar -czf package.tar.gz package
    popd
    
    
    # Publish package via http/apache for the vehicles to download
    if [ ! -d "/var/www/html/raspberry" ]; then
        sudo mkdir -p "/var/www/html/raspberry"
        sudo chmod a+rwx "/var/www/html/raspberry"
    fi
    rm -f /var/www/html/raspberry/package.tar.gz
    #cp ./build_arm_sim/package.tar.gz /var/www/html/raspberry  # For onboard simulation
    cp ./build_arm/package.tar.gz /var/www/html/raspberry      # Normal case
    rm -f /var/www/html/raspberry/DDS_DOMAIN
    echo $DDS_DOMAIN >/var/www/html/raspberry/DDS_DOMAIN
    
    
    
    export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
    export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598
    
    rm -f /var/www/html/raspberry/DDS_INITIAL_PEER
    echo $DDS_INITIAL_PEER >/var/www/html/raspberry/DDS_INITIAL_PEER
fi
