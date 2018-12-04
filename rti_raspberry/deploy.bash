#!/bin/bash

# Argument 1: IP/hostname
# Argument 2: Vehicle ID

clear
while true; do ping -c 1 -W 0.2 -w 0.2 $1 > /dev/null && break; echo "No ping response, waiting."; sleep 1; done
sleep 0.1

sshpass -p t4nxAdDwrgqn ssh -t pi@$1 'sudo killall vehicle_rpi_firmware'
sleep 0.1

echo "Uploading"
sshpass -p t4nxAdDwrgqn scp build/vehicle_rpi_firmware pi@$1:/tmp
sleep 0.1

echo "Running"
sshpass -p t4nxAdDwrgqn ssh -t pi@$1 "sudo /tmp/vehicle_rpi_firmware $2"


#sshpass -p t4nxAdDwrgqn ssh -t pi@192.168.1.109 'sudo /home/pi/start_tmp_firmware.sh'
#sleep 1

#sshpass -p t4nxAdDwrgqn ssh -t pi@192.168.1.109 'sudo tail -f /tmp/vehicle_rpi_firmware_log'
