#!/bin/bash

clear
clear

if [ ! -d "build" ]; then
    mkdir build
fi

if [ ! -d "build/rti" ]; then
    mkdir build/rti
    rtiddsgen -language C++11 -d ./build/rti/ ../dds_idl/Parameter.idl
fi


cd build
cmake .. 
make -j8
cd ..

build/unittest