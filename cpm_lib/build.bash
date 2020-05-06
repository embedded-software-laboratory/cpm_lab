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
make -C $DIR/build -j$(nproc) && $DIR/build/unittest
cd $DIR

# Publish cpm_library package via http/apache for the HLCs to download
if [ ! -d "/var/www/html/nuc" ]; then
    sudo mkdir -p "/var/www/html/nuc"
    sudo chmod a+rwx "/var/www/html/nuc"
fi
rm -rf $DIR/cpm_library_package
mkdir $DIR/cpm_library_package
cp -R $DIR/../dds_idl/ $DIR/cpm_library_package
cp -R $DIR/dds_idl_matlab/ $DIR/cpm_library_package
cp $DIR/build/libcpm.so $DIR/cpm_library_package
tar -czvf cpm_library_package.tar.gz -C $DIR/ cpm_library_package
rm -f /var/www/html/nuc/cpm_library_package.tar.gz
mv $DIR/cpm_library_package.tar.gz /var/www/html/nuc
