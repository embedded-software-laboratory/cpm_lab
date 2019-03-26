#!/bin/bash

mkdir dds

# Generate IDL C++
find ../dds_idl/ -type f | xargs -n 1 rtiddsgen -replace -language C++11 -d ./dds/

mkdir include/cpm/dds

# Copy headers to public inclues
(cd dds;find -type f) | grep \\.h | xargs -n 1 -I ARG cp dds/ARG include/cpm/dds/
