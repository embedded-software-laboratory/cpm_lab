#!/bin/bash

sshpass -p password ssh -o StrictHostKeyChecking=no -t pi@$1 'sudo reboot now'
