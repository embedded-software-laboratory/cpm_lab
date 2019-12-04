#!/bin/bash

# DIR holds the location of build.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Check if apache is installed
[ ! -d "/var/www/html/" ] && echo "apache2 not installed..aborting, install via 'sudo apt-get install apache2'" && exit

sudo mkdir -p "/var/www/html/nuc"
mkdir -p $DIR/build

if [ ! -d "dds" ]; then
    ./rtigen.bash
fi

cd $DIR/build
cmake ..
make -C $DIR/build -j8

# Publish cpm_library package via http/apache for the HLCs to download
rm -rf $DIR/cpm_library_package
mkdir $DIR/cpm_library_package
cp -R $DIR/../dds_idl/ $DIR/cpm_library_package
cp $DIR/../cpm_lib/build/libcpm.so $DIR/cpm_library_package
tar -czvf cpm_library_package.tar.gz $DIR/cpm_library_package
sudo rm -f /var/www/html/nuc/cpm_library_package.tar.gz
sudo mv $DIR/cpm_library_package.tar.gz /var/www/html/nuc

$DIR/build/unittest
