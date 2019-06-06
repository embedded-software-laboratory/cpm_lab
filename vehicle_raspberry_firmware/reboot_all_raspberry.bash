#!/bin/bash


for ip_file in package/vehicle_id_map/*; 
do
    ip="$(basename -- $ip_file)"
    echo "Rebooting $ip"
    sshpass -p t4nxAdDwrgqn ssh -o StrictHostKeyChecking=no -t -t pi@$ip 'sudo reboot now' &
    sleep 1
done


