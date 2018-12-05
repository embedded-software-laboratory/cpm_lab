#!/bin/bash

clear

rm -rf build
mkdir build

rm -rf build_sim
mkdir build_sim


cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j4
cd ..


cd build_sim
cmake .. -DBUILD_SIMULATION=ON
make -j4
cd ..
