#!/bin/bash

if [ ! -d "dds" ]; then
    mkdir dds

    # Generate IDL C++
    find ../dds_idl/ -type f | xargs -n 1 rtiddsgen -language C++11 -d ./dds/
fi

if [ ! -d "include/cpm/dds" ]; then
    mkdir include/cpm/dds

    # Copy headers to public inclues
    (cd ../dds_idl/; find . -type f) | sed -r 's/..(.*).idl/dds\/\1.hpp/' | xargs -n 1 -I ARG cp ARG include/cpm/dds/
fi
