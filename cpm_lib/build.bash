#!/bin/bash

# DIR holds the location of build.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

if [ ! -d "dds" ]; then
    echo "Generating C++ IDL files..."
    ./rtigen.bash
fi

if [ ! -d "dds_idl_matlab" ]; then
    echo "Generating Matlab IDL files..."
    matlab -sd "./" -batch "rtigen_matlab"
fi

mkdir -p $DIR/build

cd $DIR/build
cmake ..
make -C $DIR/build -j8 && $DIR/build/unittest
cd $DIR

if [ -d "/var/www/html/" ]; then
    # Apache installed
    # Publish cpm_library package via http/apache for the HLCs to download
    sudo mkdir -p "/var/www/html/nuc"
    rm -rf $DIR/cpm_library_package
    mkdir $DIR/cpm_library_package
    cp -R $DIR/../dds_idl/ $DIR/cpm_library_package
    cp -R $DIR/dds_idl_matlab/ $DIR/cpm_library_package
    cp $DIR/libcpm.so $DIR/cpm_library_package
    tar -czvf cpm_library_package.tar.gz -C $DIR/ cpm_library_package
    sudo rm -f /var/www/html/nuc/cpm_library_package.tar.gz
    sudo mv $DIR/cpm_library_package.tar.gz /var/www/html/nuc
else
    echo "WARNING: apache2 not installed - vehicles / hlcs will not be able to download if this is the master pc, install via 'sudo apt-get install apache2'"
fi