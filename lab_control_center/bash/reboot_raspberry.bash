#!/bin/bash

sshpass -p $RPIPWD ssh -o StrictHostKeyChecking=no -t pi@$1 'sudo reboot now'
