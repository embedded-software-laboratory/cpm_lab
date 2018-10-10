#!/bin/bash
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j4
cd ..
