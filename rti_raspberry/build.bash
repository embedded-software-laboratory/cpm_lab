#!/bin/bash

rm -rf rtidds
mkdir rtidds
rm -rf build
mkdir build

rtiddsgen -language C++11 ../dds_idl/Pose2D.idl -example x64Linux3gcc5.4.0 -d ./rtidds/

cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j4
cd ..
