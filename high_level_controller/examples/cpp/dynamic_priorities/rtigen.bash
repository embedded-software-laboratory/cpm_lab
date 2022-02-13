#!/bin/bash
set -e

mkdir -p ./src/dds_idl_cpp

# Generate IDL C++
find src/dds_idl/ -type f | xargs -n 1 rtiddsgen -replace -legacyPlugin -language C++11 -d ./src/dds_idl_cpp/ -I ../../../../cpm_lib/dds_idl/

# Copy headers to public inclues
mkdir -p include/dds
(cd src/dds_idl_cpp;find -type f) | grep \\.h | xargs -n 1 -I ARG cp src/dds_idl_cpp/ARG include/dds/
