#!/bin/bash

if [ $# -eq 0 ]
then
    printf "Usage (Example):\nCreate Vehicle in LCC first, then:\n./run_distributed --vehicle_ids=1,2,3,4\n"
    exit 1
fi

for arg in "$@"
do
    case $arg in
        -id*|--vehicle_ids=*)
            vehicle_ids="${arg#*=}"
            shift
            ;;
        *)
            ;;
    esac
done

printf "vehicle_ids: ${vehicle_ids}\n"
printf "DDS_DOMAIN: ${DDS_DOMAIN}\n"

# # Sollten eigentlich auch in environment_variables_local.bash gesetzt werden
# IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
# DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

# $DDS_DOMAIN wird irgendwo gesetzt, aber wann?

cleanup(){
    printf "Cleaning up ... "
    tmux kill-session -tdds_record
    for vehicle_id in ${vehicle_ids//,/ }
    do
        tmux kill-session -thigh_level_controller${vehicle_id}
    done
    printf "Done.\n"
}
trap cleanup EXIT

tmux new-session -d -s "dds_record" rtirecordingservice -cfgFile /tmp/rti_recording_config.xml -cfgName cpm_recorder

printf "Starting HLCs ...\n"
for vehicle_id in ${vehicle_ids//,/ }
do
    # Sleep gives enough time for other controllers to start
    # This is a band-aid fix and should be changed
    tmux new-session -d -s "high_level_controller${vehicle_id}" ". ~/dev/software/lab_control_center/bash/environment_variables_local.bash;cd /home/dev/dev/software/high_level_controller/examples/cpp/decentral_routing/build/;./decentral_routing --node_id=high_level_controller${vehicle_id} --simulated_time=false --vehicle_ids=${vehicle_id} --dds_domain=${DDS_DOMAIN} --dds_initial_peer=${DDS_INITIAL_PEER}  >~/dev/lcc_script_logs/stdout_hlc${vehicle_id}.txt 2>~/dev/lcc_script_logs/stderr_hlc${vehicle_id}.txt"
    printf "\tStarting high_level_controller${vehicle_id}.\n"
    sleep 10
done
printf "Done.\n"

printf "To abort, press Ctrl+C\n"

while true
do
    sleep 1
done
