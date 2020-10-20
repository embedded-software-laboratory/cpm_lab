#!/bin/bash

if [ $# -eq 0 ]
then
    printf "Usage (Example):\nCreate Vehicle in LCC first, then:\n./run_distributed --vehicle_ids=1,2,3,4\n"
    exit 1
fi

middleware=false

for arg in "$@"
do
    case $arg in
        -id*|--vehicle_ids=*)
            vehicle_ids="${arg#*=}"
            shift
            ;;
        --middleware)
            middleware=true
            shift
            ;;
        *)
            ;;
    esac
done

printf "vehicle_ids: ${vehicle_ids}\n"
printf "DDS_DOMAIN: ${DDS_DOMAIN}\n"

. ~/dev/software/lab_control_center/bash/environment_variables_local.bash

cleanup(){
    printf "Cleaning up ... "
    tmux kill-session -tdds_record
    tmux kill-session -tmiddleware_${vehicle_ids}
    for vehicle_id in ${vehicle_ids//,/ }
    do
        tmux kill-session -thigh_level_controller${vehicle_id}
    done
    # Kill all children
    kill $(jobs -p)
    printf "Done.\n"
}
trap cleanup EXIT

# Recording Service cannot be started like this, because the config file needs
# to be created from a template first
#tmux new-session -d -s "dds_record" rtirecordingservice -cfgFile /tmp/rti_recording_config.xml -cfgName cpm_recorder

sleep 3

# Start middleware with simulated time = false
# Is the dds_initial_peer parameter required? It seems unused
if $middleware;
then
    printf "Starting Middleware ...\n"
    tmux new-session -d -s "middleware_${vehicle_ids}" ". ~/dev/software/lab_control_center/bash/environment_variables_local.bash;cd /home/dev/dev/software/middleware/build/;./middleware --node_id=middleware_${vehicle_ids} --simulated_time=false --vehicle_ids=${vehicle_ids} domain_number=${DDS_DOMAIN} --dds_initial_peer=${DDS_INITIAL_PEER}  >~/dev/lcc_script_logs/stdout_middleware_${vehicle_ids}.txt 2>~/dev/lcc_script_logs/stderr_middleware_${vehicle_ids}.txt"
fi

printf "Starting HLCs ...\n"
cd build
for vehicle_id in ${vehicle_ids//,/ }
do
    # Sleep gives enough time for other controllers to start
    # This is a band-aid fix and should be changed
    #tmux new-session -d -s "high_level_controller${vehicle_id}" ". ~/dev/software/lab_control_center/bash/environment_variables_local.bash;cd /home/dev/dev/software/high_level_controller/examples/cpp/decentral_routing/build/;./decentral_routing --node_id=high_level_controller${vehicle_id} --simulated_time=false --vehicle_ids=${vehicle_id} --middleware=true --dds_domain=${DDS_DOMAIN} --dds_initial_peer=${DDS_INITIAL_PEER}  >~/dev/lcc_script_logs/stdout_hlc${vehicle_id}.txt 2>~/dev/lcc_script_logs/stderr_hlc${vehicle_id}.txt"

    if [ $vehicle_id -eq 5 ]; then
        gdb --args ./decentral_routing --node_id=high_level_controller${vehicle_id} --simulated_time=false --vehicle_ids=${vehicle_id} --middleware=true --dds_domain=${DDS_DOMAIN} --dds_initial_peer=${DDS_INITIAL_PEER}
    else
        ./decentral_routing --node_id=high_level_controller${vehicle_id} --simulated_time=false --vehicle_ids=${vehicle_id} --middleware=true --dds_domain=${DDS_DOMAIN} --dds_initial_peer=${DDS_INITIAL_PEER}  >~/dev/lcc_script_logs/stdout_hlc${vehicle_id}.txt 2>~/dev/lcc_script_logs/stderr_hlc${vehicle_id}.txt&
    fi
    printf "\tStarting high_level_controller${vehicle_id}.\n"
    #sleep 10
done
printf "Done.\n\n"

printf "To abort, press Ctrl+C\n"

# This displays all log files of hlcs in lcc_script_logs
# This may be more than we actually created
printf "Displaying stdout and stderr of all started High Level Controller\n"
tail -f ~/dev/lcc_script_logs/*.txt

while true
do
    sleep 1
done
