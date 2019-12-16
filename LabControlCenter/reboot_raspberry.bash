#!/bin/bash

sshpass -p 12345678 ssh -o StrictHostKeyChecking=no -t pi@$1 'sudo reboot now'
