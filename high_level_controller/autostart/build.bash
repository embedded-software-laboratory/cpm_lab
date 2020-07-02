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
mkdir autostart_package
cp ${BASH_DIR}/build/autostart ./autostart_package
cp ${BASH_DIR}/build/download_error_logger ./autostart_package
tar -czvf autostart_package.tar.gz autostart_package
rm -f /var/www/html/nuc/autostart_package.tar.gz
cp ./autostart_package.tar.gz /var/www/html/nuc
