#!/bin/bash
# exit when any command fails
set -e

# Funktioniert leider nicht aufgrund der absoluten Pfade im makefile, schlage vor wir packen die fertige library ins git
# cd matlab_dll/planTrajectory
# make -f planTrajectory_rtw.mk

# cd ../..
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ..
