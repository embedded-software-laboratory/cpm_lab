#!/bin/bash

# Argument 1: IP/hostname
# Argument 2: Vehicle ID

clear
while true; do ping -c 1 -W 0.2 -w 0.2 $1 > /dev/null && break; echo "No ping response, waiting."; sleep 1; done
sleep 0.1

sshpass -p t4nxAdDwrgqn ssh -o StrictHostKeyChecking=no -t pi@$1 'sudo killall vehicle_rpi_firmware'
sleep 0.1

echo "Uploading"
sshpass -p t4nxAdDwrgqn scp -o StrictHostKeyChecking=no build/vehicle_rpi_firmware pi@$1:/tmp
sshpass -p t4nxAdDwrgqn scp -o StrictHostKeyChecking=no ../../cpm_base/cpm_lib/build_arm/libcpm.so pi@$1:/tmp
sleep 0.1

echo "Running"
sshpass -p t4nxAdDwrgqn ssh -o StrictHostKeyChecking=no -t pi@$1 "sudo LD_LIBRARY_PATH=/tmp /tmp/vehicle_rpi_firmware $2"
