#!/bin/bash

clear
clear

if [ ! -d "build" ]; then
    mkdir build
fi

if [ ! -d "dds" ]; then
    ./rtigen.bash
fi


cd build
cmake .. 
make -j8 && ./unittest
cd ..
