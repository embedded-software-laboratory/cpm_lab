#!/bin/bash

IDS=$'
01
02
03
04
05
06
07
08
09
10
11
12
13
14
15
16
17
18
19'

folder="logs/$(date +%Y_%m_%d_%H_%M_%S)"
mkdir -p $folder

for id in $IDS;
do
    echo "Retrieve log from vehicle $id"
    sshpass -p "cpmcpmcpm" scp pi@192.168.1.1$id:/tmp/package/Log* $folder/log_veh_$id &
    sleep 0.1
done

# Run last command in foreground so that hopefully all output is given until the shell allows the next command
echo "Retrieve log from vehicle 20"
sshpass -p "cpmcpmcpm" scp pi@192.168.1.120:/tmp/package/Log* log_veh_20