#!/bin/bash

clear
clear

if [ ! -d "build" ]; then
    mkdir build
fi

./rtigen.bash


cd build
cmake .. 
make -j8
cd ..

build/unittest