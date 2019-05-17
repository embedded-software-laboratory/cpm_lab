#!/bin/bash

# This script is installed on each raspberry under "/root/".
# It downloads the raspberry software from the master PC
# and runs it.


# Add the following line to "/etc/rc.local" to enable auto start. Install tmux first.
# tmux new-session -d -s "bootloader_raspberry" "/root/bootloader_raspberry.bash"



cd /tmp
rm package.tar.gz
wget http://192.168.1.249/raspberry/package.tar.gz
tar -xzvf package.tar.gz
cd package
bash start.bash
