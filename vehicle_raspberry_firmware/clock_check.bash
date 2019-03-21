#!/bin/bash

sshpass -p 'raspberry' ssh -t pi@192.168.1.109 'date +%s.%N' 
date +%s.%N