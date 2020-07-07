#!/bin/bash

mkdir build
cd build

cmake .. 
make -j$(nproc)
cd ..

# Publish autostart package via http/apache for the HLCs to download - TODO: Change tmp-version to local version after merge, similar to middleware build
cd /tmp
mkdir autostart_package
cp ~/dev/software/high_level_controller/autostart/build/autostart ./autostart_package
cp ~/dev/software/high_level_controller/autostart/build/download_error_logger ./autostart_package
tar -czf autostart_package.tar.gz autostart_package
rm -f /var/www/html/nuc/autostart_package.tar.gz
cp ./autostart_package.tar.gz /var/www/html/nuc
