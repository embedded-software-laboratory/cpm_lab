#!/bin/bash

mkdir build
cd build

cmake .. 
make -j8
cd ..

# Publish autostart package via http/apache for the HLCs to download
cd /tmp
mkdir autostart_package
cp /home/cpm/dev/software/hlc/autostart/build/autostart ./autostart_package
tar -czvf autostart_package.tar.gz autostart_package
rm -f /var/www/html/nuc/autostart_package.tar.gz
cp ./autostart_package.tar.gz /var/www/html/nuc