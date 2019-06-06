#!/bin/bash


mkdir build_arm
mkdir build_arm_sim
mkdir build_x64_sim


pushd build_x64_sim
cmake .. -DBUILD_ARM=OFF -DBUILD_SIMULATION=ON
make -j20
popd


pushd build_arm_sim
cmake .. -DBUILD_ARM=ON -DBUILD_SIMULATION=ON -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j20
popd


pushd build_arm
cmake .. -DBUILD_ARM=ON -DBUILD_SIMULATION=OFF -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j20
popd

exit


cp build/vehicle_rpi_firmware package
cp ../../cpm_base/cpm_lib/build_arm/libcpm.so package
tar -czvf package.tar.gz package
rm /var/www/html/raspberry/package.tar.gz
cp ./package.tar.gz /var/www/html/raspberry