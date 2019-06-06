#!/bin/bash


mkdir build_arm
mkdir build_arm_sim
mkdir build_x64_sim


# Build for simulation on desktop
pushd build_x64_sim
cmake .. -DBUILD_ARM=OFF -DBUILD_SIMULATION=ON
make -j20
popd


# Build for simulation on Raspberry
pushd build_arm_sim
cmake .. -DBUILD_ARM=ON -DBUILD_SIMULATION=ON -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j20
cp -R ../package/ .
cp vehicle_rpi_firmware package
cp ../../../cpm_base/cpm_lib/build_arm/libcpm.so package
tar -czvf package.tar.gz package
popd


# Build for normal operation on Raspberry
pushd build_arm
cmake .. -DBUILD_ARM=ON -DBUILD_SIMULATION=OFF -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j20
cp -R ../package/ .
cp vehicle_rpi_firmware package
cp ../../../cpm_base/cpm_lib/build_arm/libcpm.so package
tar -czvf package.tar.gz package
popd



# Publish package via http/apache for the vehicles to download
rm /var/www/html/raspberry/package.tar.gz
cp ./build_arm_sim/package.tar.gz /var/www/html/raspberry  # For onboard simulation
#cp ./build_arm/package.tar.gz /var/www/html/raspberry      # Normal case