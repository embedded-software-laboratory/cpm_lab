#!/bin/bash

if [ ! -d "build_arm" ]; then
    mkdir build_arm
fi

if [ ! -d "dds" ]; then
    ./rtigen.bash
fi


cd build_arm
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake -DBUILD_ARM=ON
make -j$(nproc)
cd ..
