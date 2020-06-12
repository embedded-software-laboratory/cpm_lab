#!/bin/bash
set -e

mkdir -p dds

# Generate IDL C++
find ../dds_idl/ -type f | xargs -n 1 rtiddsgen -replace -legacyPlugin -language C++11 -d ./dds/

# Copy headers to public inclues
mkdir -p include/cpm/dds
(cd dds;find -type f) | grep \\.h | xargs -n 1 -I ARG cp dds/ARG include/cpm/dds/
