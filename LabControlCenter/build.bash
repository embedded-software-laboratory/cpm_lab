#!/bin/bash

clear
rm -rf build
mkdir build
rm -rf rtidds
mkdir rtidds

rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/VehicleState.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/TimeStamp.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/Header.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/VehicleCommandDirect.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/Pose2D.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/VehicleCommandTrajectory.idl
rtiddsgen -language C++11 -d ./rtidds/ ../dds_idl/VehicleCommandSpeedCurvature.idl

cd build
cmake .. 
make -j4
cd ..
