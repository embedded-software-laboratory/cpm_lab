#!/bin/bash
#Get command line arguments
for i in "$@"
do
case $i in
    -sp=*|--script_path=*)
    script_path="${i#*=}"
    shift # past argument=value
    ;;
    -sn=*|--script_name=*)
    script_name="${i#*=}"
    shift # past argument=value
    ;;
    -vi=*|--vehicle_ids=*)
    vehicle_ids="${i#*=}"
    shift # past argument=value
    ;;
    -va=*|--vehicle_amount=*)
    vehicle_amount="${i#*=}"
    shift # past argument=value
    ;;
    -hi=*|--hlc_ids=*)
    hlc_ids="${i#*=}"
    shift # past argument=value
    ;;
    -st=*|--simulated_time=*)
    simulated_time="${i#*=}"
    shift # past argument=value
    ;;
    -pw=*|--password=*)
    password="${i#*=}"
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

# Test values -> remove this later on
# script_path=matlab/platoon_example
# script_name=main_vehicle_ids
# vehicle_ids=14,13
# simulated_time=true

#Check for existence of required command line arguments
if [ -z "$script_path" ] || [ -z "$script_name" ] || ( [ -z "$vehicle_ids" ] && [ -z "$vehicle_amount" ] ) || [ -z "$simulated_time" ] || [ -z "$hlc_ids" ] || [ -z "$password" ]
then
      echo "Usage: bash launch_*.bash --script_path=... --script_name=... --vehicle_amount=... --simulated_time=..."
      echo "Or: bash launch_*.bash --script_path=... --script_name=... --vehicle_ids=... --simulated_time=..."
      exit 1
fi

#If vehicle amount was set, create vehicle id list in vehicle_ids from that, style: 1,...,vehicle_amount
if !([ -z "$vehicle_amount" ])
then
    vehicle_ids=$(seq -s, 1 1 ${vehicle_amount})
fi

# Get vehicle and HLC ID array
IFS=',' read -r -a vehicle_array <<< "$vehicle_ids"
IFS=',' read -r -a hlc_array <<< "$hlc_ids"

exit_script() {
    tmux kill-session -t "LabControlCenter"
    tmux kill-session -t "rticlouddiscoveryservice"
    tmux kill-session -t "BaslerLedDetection"
    tmux kill-session -t "ips_pipeline"

    # Stop HLCs and vehicles
    for index in "${!hlc_array[@]}"
    do
        ip=$(printf "192.168.1.2%02d" ${hlc_array[index]})
        id=${vehicle_array[index]}
        echo $ip
        sshpass -p $password ssh -t controller@$ip 'bash /tmp/software/hlc/stop.bash'

        #tmux kill-session -t "vehicle_${id}"
    done
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

# Publish package via http/apache for the NUCs to download -> in build_all? Was ist mit HLC scripts?
#   1. make middleware
# pushd hlc/middleware
# bash build.bash
# popd
#   2. create tar
mkdir nuc_apache_package
pushd nuc_apache_package
tar -czvf nuc_package.tar.gz ../../software/hlc ../../cpm_base/cpm_lib/build/libcpm.so ../../cpm_base/dds_idl
popd
#   3. publish package
rm -f /var/www/html/nuc/nuc_package.tar.gz
cp ./nuc_apache_package/nuc_package.tar.gz /var/www/html/nuc
rm -f /var/www/html/nuc/DDS_DOMAIN
echo $DDS_DOMAIN >/var/www/html/nuc/DDS_DOMAIN
rm -f /var/www/html/nuc/DDS_INITIAL_PEER
echo $DDS_INITIAL_PEER >/var/www/html/nuc/DDS_INITIAL_PEER

# Iterate over HLC IDs, get corresponding vehicle IDs
for index in "${!hlc_array[@]}"
do
    ip=$(printf "192.168.1.2%02d" ${hlc_array[index]})
    id=${vehicle_array[index]}

    echo $ip
    # Start download / start script on NUCs to start HLC and middleware
    sshpass -p $password rsync -v -e 'ssh -o StrictHostKeyChecking=no -p 22' ./hlc/apache_start.bash controller@$ip:/tmp/
    # sshpass -p $password rsync -v -e 'ssh -o StrictHostKeyChecking=no -p 22' /tmp/hlc/ controller@$ip:/tmp/
    sshpass -p $password ssh -t controller@$ip 'bash /tmp/apache_start.bash' "${script_path} ${script_name} ${id} ${simulated_time}"

    # Start vehicle (here for test purposes, later: use real vehicles / position them correctly)
    #tmux new-session -d -s "vehicle_${id}" "cd ./vehicle_raspberry_firmware/;bash run_w_flexible_domain.bash ${id} 3 ${simulated_time}"
done

sleep infinity