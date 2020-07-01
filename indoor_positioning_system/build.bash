#!/bin/bash
# exit when any command fails
set -e

mkdir build
cd build
cmake .. 
make -j$(nproc) && ./unittest
cd ..
