#!/bin/bash
# input: vehicle ID padded with leading 0 to two digits, e.g. 
# bash rebooot_raspberry.bash 08

sshpass -p cpmcpmcpm ssh -o StrictHostKeyChecking=no -t pi@192.168.1.1$1 'sudo reboot now'