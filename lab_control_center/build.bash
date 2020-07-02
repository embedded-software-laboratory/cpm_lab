#!/bin/bash
# exit when any command fails
set -e

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


mkdir -p build

cd build
cmake .. -DSIMULATION=$SIMULATION 
make -j8
cd ..

# Create launcher link to LCC
if [ -d "${HOME}/.local/share/applications/" ]; then
    escaped_home=$(printf '%s\n' "$HOME" | sed 's:[][\/.^$*]:\\&:g')
    sed 's/~/'"$escaped_home"'/g' lab-control-center.desktop > $HOME/.local/share/applications/lab-control-center.desktop
fi