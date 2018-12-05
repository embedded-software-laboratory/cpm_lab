#!/bin/bash

clear

mkdir build

cd build
cmake .. 
make -j4
cd ..
