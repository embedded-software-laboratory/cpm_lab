#!/bin/bash

exit_script() {
    tmux kill-session -t "LabControlCenter"
    tmux kill-session -t "rticlouddiscoveryservice"
    tmux kill-session -t "BaslerLedDetection"
    tmux kill-session -t "ips_pipeline"
    tmux kill-session -t "test_loop"
    
    trap - SIGINT SIGTERM # clear the trap
}


export IP_SELF="$(hostname -I)"
export IP_SELF="$(echo $IP_SELF)"
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

trap exit_script SIGINT SIGTERM

# Start local software (WARNING: domain hardcoded right now)
tmux new-session -d -s "rticlouddiscoveryservice" "rticlouddiscoveryservice -transport 25598  >stdout_rticlouddiscoveryservice.txt 2>stderr_rticlouddiscoveryservice.txt"
sleep 3
tmux new-session -d -s "LabControlCenter" "(cd LabControlCenter;./build/LabControlCenter --dds_domain=21 --simulated_time=${simulated_time} --dds_initial_peer=$DDS_INITIAL_PEER >stdout.txt 2>stderr.txt)"
tmux new-session -d -s "BaslerLedDetection" "(cd ips2;./build/BaslerLedDetection --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER >stdout_led_detection.txt 2>stderr_led_detection.txt)"
tmux new-session -d -s "ips_pipeline" "(cd ips2;./build/ips_pipeline --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER >stdout_ips.txt 2>stderr_ips.txt)"
tmux new-session -d -s "test_loop" "(cd controller_test_loop;./build/controller_test_loop --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER >stdout_test_loop.txt 2>stderr_test_loop.txt)"

sleep infinity