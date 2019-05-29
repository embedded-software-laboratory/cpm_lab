#!/bin/bash

sshpass -p t4nxAdDwrgqn ssh -o StrictHostKeyChecking=no -t pi@$1 'sudo reboot now'