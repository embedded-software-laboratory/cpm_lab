#!/bin/bash
# exit when any command fails
set -e

if [ ! -d "build_arm" ]; then
    mkdir build_arm
fi

if [ ! -d "dds_idl_cpp" ]; then
    ./rtigen.bash
fi


cd build_arm
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake -DBUILD_ARM=ON
make -j$(nproc)
cd ..
