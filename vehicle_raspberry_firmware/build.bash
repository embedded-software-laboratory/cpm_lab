#!/bin/bash


mkdir build_arm
mkdir build_x64_sim


pushd build_x64_sim
cmake .. -DBUILD_X64=ON -DBUILD_SIMULATION=ON
make -j20
popd


pushd build_arm
cmake .. -DBUILD_X64=OFF -DBUILD_SIMULATION=OFF -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j20
popd

exit


cp build/vehicle_rpi_firmware package
cp ../../cpm_base/cpm_lib/build_arm/libcpm.so package
tar -czvf package.tar.gz package
rm /var/www/html/raspberry/package.tar.gz
cp ./package.tar.gz /var/www/html/raspberry