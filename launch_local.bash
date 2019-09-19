#!/bin/bash
exit_script() {
    tmux kill-session -t "LabControlCenter"
    tmux kill-session -t "rticlouddiscoveryservice"
    tmux kill-session -t "BaslerLedDetection"
    tmux kill-session -t "ips_pipeline"
    trap - SIGINT SIGTERM # clear the trap
}


export IP_SELF="$(hostname -I)"
export IP_SELF="$(echo $IP_SELF)"
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

trap exit_script SIGINT SIGTERM

tmux new-session -d -s "rticlouddiscoveryservice" "rticlouddiscoveryservice -transport 25598  >stdout_rticlouddiscoveryservice.txt 2>stderr_rticlouddiscoveryservice.txt"
sleep 3
tmux new-session -d -s "LabControlCenter" "(cd LabControlCenter;./build/LabControlCenter --dds_domain=3 --dds_initial_peer=$DDS_INITIAL_PEER >stdout.txt 2>stderr.txt)"

sleep infinity