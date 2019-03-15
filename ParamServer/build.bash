#!/bin/bash

clear

mkdir build

cd build
cmake .. 
make -j8 && ./unittest
cd ..
