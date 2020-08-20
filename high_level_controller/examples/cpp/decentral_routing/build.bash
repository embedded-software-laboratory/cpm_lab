#!/bin/bash
# exit when any command fails
set -e

mkdir -p build
cd build
cmake .. 
make -j$(nproc)
cd ..
