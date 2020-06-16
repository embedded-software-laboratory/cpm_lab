#!/bin/bash

mkdir -p build

# Copy local communication QoS, use correct IP
IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
sed -e "s/TEMPLATE_IP/${IP_SELF}/g" \
<./QOS_LOCAL_COMMUNICATION.xml.template \
>./build/QOS_LOCAL_COMMUNICATION.xml

cd build
cmake .. 
make -j$(nproc)

# Publish middleware package via http/apache for the HLCs to download
if [ ! -d "/var/www/html/nuc" ]; then
    sudo mkdir -p "/var/www/html/nuc"
    sudo chmod a+rwx "/var/www/html/nuc"
fi
# DIR holds the location of build.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $DIR
rm -rf middleware_package
mkdir middleware_package
cp ~/dev/software/middleware/build/middleware ./middleware_package
cp ~/dev/software/middleware/QOS_LOCAL_COMMUNICATION.xml.template ./middleware_package
tar -czvf middleware_package.tar.gz middleware_package
rm -f /var/www/html/nuc/middleware_package.tar.gz
cp ./middleware_package.tar.gz /var/www/html/nuc
rm -rf middleware_package
rm -rf middleware_package.tar.gz

# Perform unittest
cd ~/dev/software/middleware/build 
./unittest
