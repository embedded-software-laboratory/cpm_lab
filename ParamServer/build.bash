#!/bin/bash
clear

# Get yaml
pushd ../..
if [ ! -d "yaml-cpp" ]; then
    git clone https://github.com/jbeder/yaml-cpp.git
    cd yaml-cpp
    mkdir build
    cd build
    cmake .. -DBUILD_SHARED_LIBS=ON
    make -j8
fi
popd

# Make the rest
mkdir build
cd build
cmake .. 
make -j8 && ./unittest
cd ..
