#!/bin/bash
#Default value for DDS domain (domain of real vehicles)
dds_domain=21

#Get command line arguments
for i in "$@"
do
case $i in
    -dd=*|--dds_domain=*)
    dds_domain="${i#*=}"
    shift # past argument=value
    ;;
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
    # -pw=*|--password=*)
    # password="${i#*=}"
    # shift # past argument=value
    # ;;
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
if [ -z "$script_path" ] || [ -z "$script_name" ] || ( [ -z "$vehicle_ids" ] && [ -z "$vehicle_amount" ] ) || [ -z "$simulated_time" ] || [ -z "$hlc_ids" ]
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

    # Stop HLCs and vehicles
    for index in "${!hlc_array[@]}"
    do
        ip=$(printf "192.168.1.2%02d" ${hlc_array[index]})
        id=${vehicle_array[index]}
        echo $ip
        sshpass ssh -t guest@$ip 'bash /tmp/software/high_level_controller/stop.bash'

        #tmux kill-session -t "vehicle_${id}"
    done
    trap - SIGINT SIGTERM # clear the trap
}


export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

trap exit_script SIGINT SIGTERM

# Start local software (WARNING: domain hardcoded right now)
tmux new-session -d -s "LabControlCenter" "(cd LabControlCenter;./build/LabControlCenter --dds_domain=${dds_domain} --simulated_time=${simulated_time} --dds_initial_peer=$DDS_INITIAL_PEER >stdout.txt 2>stderr.txt)"

# Publish package via http/apache for the NUCs to download -> in build_all? Was ist mit HLC scripts?
#   1. make middleware
# pushd middleware
# bash build.bash
# popd
#   2. create tar
mkdir nuc_apache_package
pushd nuc_apache_package
tar -czvf nuc_package.tar.gz ../../software/high_level_controller ../../software/cpm_lib/build/libcpm.so ../../software/dds_idl
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
    sshpass rsync -v -e 'ssh -o StrictHostKeyChecking=no -p 22' ./high_level_controller/apache_start.bash guest@$ip:/tmp/
    # sshpass rsync -v -e 'ssh -o StrictHostKeyChecking=no -p 22' /tmp/high_level_controller/ guest@$ip:/tmp/
    sshpass ssh -t guest@$ip 'bash /tmp/apache_start.bash' "${script_path} ${script_name} ${id} ${simulated_time}"

    # Start vehicle (here for test purposes, later: use real vehicles / position them correctly)
    #tmux new-session -d -s "vehicle_${id}" "cd ./mid_level_controller/;bash run_w_flexible_domain.bash ${id} 21 ${simulated_time}"
done

sleep infinity