#!/bin/bash


# Get yaml
pushd ../..
if [ ! -d "yaml-cpp" ]; then
    git clone https://github.com/jbeder/yaml-cpp.git
    cd yaml-cpp
    mkdir build
    cd build
    cmake .. -DBUILD_SHARED_LIBS=ON
    make -j$(nproc)
fi
popd


mkdir build

cd build
cmake .. -DSIMULATION=$SIMULATION
make -j8
cd ..
