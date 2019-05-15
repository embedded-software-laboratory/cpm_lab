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

