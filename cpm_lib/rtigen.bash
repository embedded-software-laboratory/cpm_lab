#!/bin/bash
set -e

mkdir -p dds_idl_cpp

# Generate IDL C++
find dds_idl/ -type f -exec rtiddsgen -replace -legacyPlugin -language C++11 -d ./dds_idl_cpp/ {} \;

# Copy headers to public inclues
mkdir -p include/cpm/dds
(cd dds_idl_cpp;find -type f) | grep \\.h | xargs -n 1 -I ARG cp dds_idl_cpp/ARG include/cpm/dds/
