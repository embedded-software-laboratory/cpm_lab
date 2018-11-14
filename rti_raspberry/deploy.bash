#!/bin/bash

clear
while true; do ping -c 1 -W 0.2 -w 0.2 192.168.1.109 > /dev/null && break; echo "No ping response, waiting."; sleep 1; done
sleep 0.1

sshpass -p 'raspberry' ssh -t pi@192.168.1.109 'sudo killall vehicle_rpi_firmware'
sleep 0.1

echo "Uploading"
sshpass -p 'raspberry' scp build/vehicle_rpi_firmware pi@192.168.1.109:/tmp
sleep 0.1

echo "Running"
sshpass -p 'raspberry' ssh -t pi@192.168.1.109 'sudo /home/pi/start_tmp_firmware.sh'
sleep 1

sshpass -p 'raspberry' ssh -t pi@192.168.1.109 'sudo tail -f /tmp/vehicle_rpi_firmware_log'
