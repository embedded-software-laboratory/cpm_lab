#!/bin/bash


mkdir build
mkdir build_sim


cd build_sim
cmake .. -DBUILD_SIMULATION=ON
make -j4
cd ..


cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j4
cd ..




cp build/vehicle_rpi_firmware package
cp ../../cpm_base/cpm_lib/build_arm/libcpm.so package
tar -czvf package.tar.gz package
rm /var/www/html/raspberry/package.tar.gz
cp ./package.tar.gz /var/www/html/raspberry