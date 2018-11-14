#!/bin/bash

clear

rm -rf rtidds
mkdir rtidds

rm -rf build
mkdir build

rm -rf build_sim
mkdir build_sim

rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/VehicleState.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/TimeStamp.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/Header.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/VehicleCommandDirect.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/Pose2D.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/VehicleCommandTrajectory.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/VehicleCommandSpeedCurvature.idl


cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain.cmake
make -j4
cd ..


cd build_sim
cmake .. -DBUILD_SIMULATION=ON
make -j4
cd ..
