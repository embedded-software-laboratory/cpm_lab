#!/bin/bash
script_path=$1
script_name=$2
vehicle_id=$3
simulated_time=$4

# Test values
script_path=matlab/platoon_example
script_name=main
vehicle_id=14,13
simulated_time=true

exit_script() {
    tmux kill-session -t "LabControlCenter"
    tmux kill-session -t "rticlouddiscoveryservice"

    # Stop HLCs and vehicles
    IFS=,
    for val in $vehicle_id;
    do
        tmux kill-session -t "hlc_${val}"
        tmux kill-session -t "middleware_${val}"
        tmux kill-session -t "vehicle_${val}"
    done
    
    trap - SIGINT SIGTERM # clear the trap
}


export IP_SELF="$(hostname -I)"
export IP_SELF="$(echo $IP_SELF)"
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

trap exit_script SIGINT SIGTERM

tmux new-session -d -s "rticlouddiscoveryservice" "rticlouddiscoveryservice -transport 25598  >stdout_rticlouddiscoveryservice.txt 2>stderr_rticlouddiscoveryservice.txt"
sleep 3
tmux new-session -d -s "LabControlCenter" "(cd LabControlCenter;./build/LabControlCenter --dds_domain=3 --simulated_time=${simulated_time} --dds_initial_peer=$DDS_INITIAL_PEER >stdout.txt 2>stderr.txt)"

IFS=,
for val in $vehicle_id;
do
    # Start HLC and middleware
    tmux new-session -d -s "middleware_${val}" "cd ./hlc/;bash middleware_start.bash ${val} ${simulated_time} &> middleware.txt"
    tmux new-session -d -s "hlc_${val}" "cd ./hlc/;bash hlc_start.bash ${script_path} ${script_name} ${val} &> hlc.txt"

    # Start vehicle
    tmux new-session -d -s "vehicle_${val}" "cd ./vehicle_raspberry_firmware/;bash run_simulated.bash ${val} 3"
done

sleep infinity