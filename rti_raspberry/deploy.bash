#!/bin/bash

clear
while true; do ping -c 1 -W 0.2 -w 0.2 192.168.1.109 > /dev/null && break; echo "No ping response, waiting."; sleep 1; done
sleep 1
echo "Uploading"
sshpass -p 'raspberry' scp build/vehicle_rpi_firmware pi@192.168.1.109:/tmp
echo "Running"
sshpass -p 'raspberry' ssh -t pi@192.168.1.109 'sudo /tmp/vehicle_rpi_firmware' 
