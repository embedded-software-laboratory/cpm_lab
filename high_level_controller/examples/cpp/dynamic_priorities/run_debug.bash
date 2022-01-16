#!/bin/bash

if [ ! -d "./build" ]
then
    printf "Please run ./build.bash first.\n"
    exit 1
fi

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
            vehicle_array=(${vehicle_ids//,/ })
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

if [ ${#vehicle_array[@]} -gt 5 ]
then
    printf "Warning: Using more than 5 vehicles during local testing \
can lead to unexpected results\n"
    # Reason: RTI DDS Participants only search for 4 other Participants
    # per domain on a single machine per default.
fi

# Get environment variables directory relative to this script's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
RELATIVE_BASH_FOLDER_DIR="${DIR}/../software/lab_control_center/bash"
ABSOLUTE_BASH_FOLDER_DIR="$(realpath "${RELATIVE_BASH_FOLDER_DIR}")"
RELATIVE_SOFTWARE_FOLDER_DIR="${DIR}/../software/"
ABSOLUTE_SOFTWARE_FOLDER_DIR="$(realpath "${RELATIVE_SOFTWARE_FOLDER_DIR}")"
RELATIVE_LOGS_FOLDER_DIR="${DIR}/../lcc_script_logs"
ABSOLUTE_LOGS_FOLDER_DIR="$(realpath "${RELATIVE_LOGS_FOLDER_DIR}")"
export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

. ${ABSOLUTE_BASH_FOLDER_DIR}/environment_variables_local.bash

cleanup(){
    printf "Cleaning up ... "

    for vehicle_id in "${vehicle_array[@]}"
    do
        tmux kill-session -t distributed_routing_${vehicle_id}
    done

    # Kill all children
    kill 0
    printf "Done.\n"
}
trap cleanup EXIT

sleep 3
cd build
for vehicle_id in "${vehicle_array[@]}"
do

        # This starts every HLC with a debugger attached and in separate tmux sessions
        # Use e.g. 'tmux attach-session -t distributed_routing_2' to start debugging
        # HLC 2
        tmux new-session -d -s "distributed_routing_${vehicle_id}" ". ~/dev/software/lab_control_center/bash/environment_variables_local.bash;cd /home/dev/dorndorf/build/;\
            gdb -ex=r --args ./dynamic_priorities \
            --node_id=high_level_controller${vehicle_id} \
            --simulated_time=false \
            --vehicle_ids=${vehicle_id} \
            --middleware=true \
	    --middleware_domain=1 \
            --dds_domain=${DDS_DOMAIN} \
            --dds_initial_peer=${DDS_INITIAL_PEER}"
    
    printf "\tStarting high_level_controller_${vehicle_id}.\n"
done
printf "Done.\n\n"

printf "To abort, press Ctrl+C\n"

# This displays all log files of hlcs in lcc_script_logs
# This may be more than we actually created
if [ $show_logs ]; then
    printf "Displaying stdout and stderr of all started High Level Controller\n"
    tail -f ${ABSOLUTE_LOGS_FOLDER_DIR}/*.txt
fi

while true
do
    sleep 1
done
