#!/bin/bash

if [ ! -d "build" ]; then
    mkdir build
fi

if [ ! -d "dds" ]; then
    ./rtigen.bash
fi

cd build
cmake .. 
make -j8
cd ..

# Publish cpm_library package via http/apache for the HLCs to download
cd /tmp
mkdir cpm_library_package
cp -R /home/cpm/dev/cpm_base/dds_idl/ ./cpm_library_package
cp /home/cpm/dev/cpm_base/cpm_lib/build/libcpm.so ./cpm_library_package
tar -czvf cpm_library_package.tar.gz cpm_library_package
rm -f /var/www/html/nuc/cpm_library_package.tar.gz
cp ./cpm_library_package.tar.gz /var/www/html/nuc

cd /home/cpm/dev/cpm_base/cpm_lib/build
./unittest
cd ..