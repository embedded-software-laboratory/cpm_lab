#!/bin/bash


for i in {1..3}
do
    nohup ./apriltag_track_video >logout_$i.txt >>logerr_$i.txt &
done

sleep 30


killall apriltag_track_video