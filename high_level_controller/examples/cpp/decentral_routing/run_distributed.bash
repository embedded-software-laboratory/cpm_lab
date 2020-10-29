#!/bin/bash

if [ $# -eq 0 ]
then
    printf "Usage (Example):\nCreate Vehicle in LCC first, then:\n./run_distributed.bash --vehicle_ids=1,2,3,4\n"
    exit 1
fi

for arg in "$@"
do
    case $arg in
        -id*|--vehicle_ids=*)
            vehicle_ids="${arg#*=}"
            shift
            ;;
        --debug)
            debug=true
            shift
            ;;
        --show_logs)
            show_logs=true
            shift
            ;;
        *)
            ;;
    esac
done

printf "vehicle_ids: ${vehicle_ids}\n"
printf "DDS_DOMAIN: ${DDS_DOMAIN}\n"

. ~/dev/software/lab_control_center/bash/environment_variables_local.bash

# Create QoS settings file for local communications
cat QOS_LOCAL_COMMUNICATION.xml.template \
| sed -e "s|TEMPLATE_IP|${IP_SELF}|g" \
>./build/QOS_LOCAL_COMMUNICATION.xml

cleanup(){
    printf "Cleaning up ... "

    tmux kill-session -tmiddleware_${vehicle_ids}

    if [ $debug ]; then
        for vehicle_id in ${vehicle_ids//,/ }
        do
            tmux kill-session -tdecentral_routing_${vehicle_id}
        done
    fi
    # Kill all children
    kill 0
    printf "Done.\n"
}
trap cleanup EXIT

sleep 3

# Start middleware
printf "Starting Middleware ...\n"
middleware_cmd="./middleware \
    --node_id=middleware_${vehicle_ids} \
    --simulated_time=false \
    --vehicle_ids=${vehicle_ids} \
    --domain_number=${DDS_DOMAIN} \
    --dds_initial_peer=${DDS_INITIAL_PEER}  \
    >~/dev/lcc_script_logs/stdout_middleware_${vehicle_ids}.txt \
    2>~/dev/lcc_script_logs/stderr_middleware_${vehicle_ids}.txt"
tmux new-session -d -s "middleware_${vehicle_ids}" ". ~/dev/software/lab_control_center/bash/environment_variables_local.bash;cd /home/dev/dev/software/middleware/build/;${middleware_cmd}"

printf "Starting HLCs ...\n"
cd build
for vehicle_id in ${vehicle_ids//,/ }
do

    # Special debug option: if debug is enabled, the 5th vehicle will be started directly in gdb
    # This is useful, when the HLC is crashing before we have time to attach a debugger
    if [  $debug ]; then
        # This starts every HLC with a debugger attached and in separate tmux sessions
        # Use e.g. 'tmux attach-session -t decentral_routing_2' to start debugging
        # HLC 2
        tmux new-session -d -s "decentral_routing_${vehicle_id}" ". ~/dev/software/lab_control_center/bash/environment_variables_local.bash;cd /home/dev/dev/software/high_level_controller/examples/cpp/decentral_routing/build/;\
            gdb -ex=r --args ./decentral_routing \
            --node_id=high_level_controller${vehicle_id} \
            --simulated_time=false \
            --vehicle_ids=${vehicle_id} \
            --dds_domain=${DDS_DOMAIN} \
            --dds_initial_peer=${DDS_INITIAL_PEER}"
    else
        # This starts the high level controller
        ./decentral_routing \
            --node_id=high_level_controller${vehicle_id} \
            --simulated_time=false \
            --vehicle_ids=${vehicle_id} \
            --dds_domain=${DDS_DOMAIN} \
            --dds_initial_peer=${DDS_INITIAL_PEER}  \
            >~/dev/lcc_script_logs/stdout_hlc${vehicle_id}.txt \
            2>~/dev/lcc_script_logs/stderr_hlc${vehicle_id}.txt&
    fi
    printf "\tStarting high_level_controller${vehicle_id}.\n"
done
printf "Done.\n\n"

printf "To abort, press Ctrl+C\n"

# This displays all log files of hlcs in lcc_script_logs
# This may be more than we actually created
if [ $show_logs ]; then
    printf "Displaying stdout and stderr of all started High Level Controller\n"
    tail -f ~/dev/lcc_script_logs/*.txt
fi

while true
do
    sleep 1
done
