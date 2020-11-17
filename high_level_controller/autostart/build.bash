#!/bin/bash
# exit when any command fails
set -e

# Get directory of bash script
BASH_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

mkdir -p build
cd build

cmake .. 
make -j$(nproc)
cd ..

# Publish autostart package via http/apache for the HLCs to download - TODO: Change tmp-version to local version after merge, similar to middleware build
cd /tmp
rm -rf autostart_package
mkdir autostart_package
cp ${BASH_DIR}/build/autostart ./autostart_package
cp ${BASH_DIR}/build/download_error_logger ./autostart_package
cp ${BASH_DIR}/lab_autostart.bash ./autostart_package # This file will be updated by lab_autostart itself, meaning that an update of that file only takes effect after a NUC restart
tar -czf autostart_package.tar.gz autostart_package
rm -f /var/www/html/nuc/autostart_package.tar.gz
cp ./autostart_package.tar.gz /var/www/html/nuc
