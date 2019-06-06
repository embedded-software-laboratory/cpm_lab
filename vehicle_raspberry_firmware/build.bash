#!/bin/bash


mkdir build_x64_sim
mkdir build_arm_sim
mkdir build_arm


pushd build_x64_sim
cmake .. -DBUILD_SIMULATION=ON -DBUILD_ARM=OFF
make -j20
popd


pushd build_arm_sim
cmake .. -DBUILD_SIMULATION=ON -DBUILD_ARM=ON -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j20
popd


pushd build_arm
cmake .. -DBUILD_SIMULATION=OFF -DBUILD_ARM=ON -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j20
popd




cp build/vehicle_rpi_firmware package
cp ../../cpm_base/cpm_lib/build_arm/libcpm.so package
tar -czvf package.tar.gz package
rm /var/www/html/raspberry/package.tar.gz
cp ./package.tar.gz /var/www/html/raspberry