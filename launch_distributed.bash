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
    tmux kill-session -t "BaslerLedDetection"
    tmux kill-session -t "ips_pipeline"

    # Stop HLCs and vehicles
    IFS=,
    for val in $vehicle_id;
    do
        ip=$(printf "192.168.1.2%02d" ${val})
        echo $ip
        sshpass -p c0ntr0ller ssh -t controller@$ip 'bash /tmp/software/hlc/stop.bash'

        tmux kill-session -t "vehicle_${val}"
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
tmux new-session -d -s "LabControlCenter" "(cd LabControlCenter;./build/LabControlCenter --dds_domain=3 --simulated_time=${simulated_time} --dds_initial_peer=$DDS_INITIAL_PEER >stdout.txt 2>stderr.txt)"
tmux new-session -d -s "BaslerLedDetection" "(cd ips2;./build/BaslerLedDetection --dds_domain=3 --dds_initial_peer=$DDS_INITIAL_PEER >stdout_led_detection.txt 2>stderr_led_detection.txt)"
tmux new-session -d -s "ips_pipeline" "(cd ips2;./build/ips_pipeline --dds_domain=3 --dds_initial_peer=$DDS_INITIAL_PEER >stdout_ips.txt 2>stderr_ips.txt)"

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

IFS=,
for val in $vehicle_id;
do
    ip=$(printf "192.168.1.2%02d" ${val})
    echo $ip
    # Start download / start script on NUCs to start HLC and middleware
    sshpass -p c0ntr0ller rsync -v -e 'ssh -o StrictHostKeyChecking=no -p 22' ./hlc/apache_start.bash controller@$ip:/tmp/
    # sshpass -p c0ntr0ller rsync -v -e 'ssh -o StrictHostKeyChecking=no -p 22' /tmp/hlc/ controller@$ip:/tmp/
    sshpass -p c0ntr0ller ssh -t controller@$ip 'bash /tmp/apache_start.bash' "${script_path} ${script_name} ${val} ${simulated_time}"

    # Start vehicle (here for test purposes, later: use real vehicles / position them correctly)
    tmux new-session -d -s "vehicle_${val}" "cd ./vehicle_raspberry_firmware/;bash run_w_flexible_domain.bash ${val} 3 ${simulated_time}"
done

sleep infinity