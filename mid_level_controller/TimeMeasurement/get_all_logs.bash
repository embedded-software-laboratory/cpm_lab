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
19
20'

folder=$(date +%Y_%m_%d_%k_%M_%S)
mkdir $folder

cd $folder

for id in $IDS;
do
    echo "Retrieve log from vehicle $id"
    sshpass -p "cpmcpmcpm" scp pi@192.168.1.1$id:/tmp/package/Log* log_veh_$id &
    sleep 0.1
done