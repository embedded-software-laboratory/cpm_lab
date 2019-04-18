#!/bin/bash

clear
clear

if [ ! -d "build" ]; then
    mkdir build
fi


cd build
cmake .. 
make -j8
cd ..
