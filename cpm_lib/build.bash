#!/bin/bash

# DIR holds the location of build.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
echo $DIR

apache_installed="true"

# Check if apache is installed
[ ! -d "/var/www/html/" ] && echo "WARNING: apache2 not installed - vehicles / hlcs will not be able to download if this is the master pc, install via 'sudo apt-get install apache2'" && apache_installed="false"

mkdir -p $DIR/build

if [ ! -d "dds" ]; then
    ./rtigen.bash
fi

cd $DIR/build
cmake ..
make -C $DIR/build -j8

if [[ $apache_installed = "true" ]]
then
    sudo mkdir -p "/var/www/html/nuc"

    # Publish cpm_library package via http/apache for the HLCs to download
    rm -rf $DIR/build/cpm_library_package
    mkdir $DIR/build/cpm_library_package
    cp -R $DIR/../dds_idl/ $DIR/build/cpm_library_package
    cp $DIR/build/libcpm.so $DIR/build/cpm_library_package
    tar -czvf cpm_library_package.tar.gz -C $DIR/build/ cpm_library_package
    sudo rm -f /var/www/html/nuc/cpm_library_package.tar.gz
    sudo mv $DIR/build/cpm_library_package.tar.gz /var/www/html/nuc

    $DIR/build/unittest
else
    echo "WARNING: apache2 not installed - vehicles / hlcs will not be able to download if this is the master pc, install via 'sudo apt-get install apache2'"
fi