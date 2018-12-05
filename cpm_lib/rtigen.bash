#!/bin/bash

if [ ! -d "dds" ]; then
    mkdir dds

    # Generate IDL C++
    find ../dds_idl/ -type f | xargs -n 1 rtiddsgen -language C++11 -d ./dds/
fi

if [ ! -d "include/cpm/dds" ]; then
    mkdir include/cpm/dds

    # Copy headers to public inclues
    (cd dds;find -type f) | grep \\.h | xargs -n 1 -I ARG cp dds/ARG include/cpm/dds/
fi
