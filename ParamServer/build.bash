#!/bin/bash
clear

# Get yaml
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make

# Make the rest
cd ..
cd ..
mkdir build

cd build
cmake .. 
make -j8 && ./unittest
cd ..
