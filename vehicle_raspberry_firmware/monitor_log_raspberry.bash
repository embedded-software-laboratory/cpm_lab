#!/bin/bash

sshpass -p t4nxAdDwrgqn ssh -o StrictHostKeyChecking=no -t pi@$1 'tail -f /tmp/package/stderr* & tail -f /tmp/package/stdout*'