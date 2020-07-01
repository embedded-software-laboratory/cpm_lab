#!/bin/bash
# exit when any command fails
set -e

simulation=0

#Get command line arguments
for arg in "$@"
do
    case $arg in
        -s|--simulation)
        simulation=1
        shift # Remove --simulation from processing
        ;;
        *)
        shift # Remove generic argument from processing
        ;;
    esac
done

mkdir build_arm
mkdir build_arm_sim
mkdir build_x64_sim


# Build for simulation on desktop
pushd build_x64_sim
cmake .. -DBUILD_ARM=OFF -DBUILD_SIMULATION=ON
make -j$(nproc)
popd

if [ $simulation == 0 ]
then
    # Build for simulation on Raspberry
    pushd build_arm_sim
    cmake .. -DBUILD_ARM=ON -DBUILD_SIMULATION=ON -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
    make -j$(nproc)
    cp -R ../package/ .
    cp vehicle_rpi_firmware package
    cp ../../cpm_lib/build_arm/libcpm.so package
    tar -czvf package.tar.gz package
    popd
    
    
    # Build for normal operation on Raspberry
    pushd build_arm
    cmake .. -DBUILD_ARM=ON -DBUILD_SIMULATION=OFF -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
    make -j$(nproc)
    cp -R ../package/ .
    cp vehicle_rpi_firmware package
    cp ../../cpm_lib/build_arm/libcpm.so package
    tar -czvf package.tar.gz package
    popd
    
    
    
    # # Publish package via http/apache for the vehicles to download
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
