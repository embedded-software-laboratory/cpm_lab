#!/bin/bash

if [ ! -d "rti" ]; then
    mkdir rti
    rtiddsgen -language C++11 -d ./rti/ ../dds_idl/Parameter.idl
    rtiddsgen -language C++11 -d ./rti/ ../dds_idl/ParameterRequest.idl
fi
