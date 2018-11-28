#!/bin/bash

clear
clear

if [ ! -d "build_arm" ]; then
    mkdir build_arm
fi

./rtigen.bash


cd build_arm
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake -DBUILD_ARM=ON
make -j8
cd ..
