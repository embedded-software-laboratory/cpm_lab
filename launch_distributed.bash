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

# Start local software
tmux new-session -d -s "rticlouddiscoveryservice" "rticlouddiscoveryservice -transport 25598  >stdout_rticlouddiscoveryservice.txt 2>stderr_rticlouddiscoveryservice.txt"
sleep 3
tmux new-session -d -s "LabControlCenter" "(cd LabControlCenter;./build/LabControlCenter --dds_domain=$DDS_DOMAIN --dds_initial_peer=$DDS_INITIAL_PEER >stdout.txt 2>stderr.txt)"
tmux new-session -d -s "BaslerLedDetection" "(cd ips2;./build/BaslerLedDetection --dds_domain=$DDS_DOMAIN --dds_initial_peer=$DDS_INITIAL_PEER >stdout_led_detection.txt 2>stderr_led_detection.txt)"
tmux new-session -d -s "ips_pipeline" "(cd ips2;./build/ips_pipeline --dds_domain=$DDS_DOMAIN --dds_initial_peer=$DDS_INITIAL_PEER >stdout_ips.txt 2>stderr_ips.txt)"

# Publish package via http/apache for the NUCs to download
#   1. make middleware
pushd middleware
bash build.bash
popd
#   2. create tar
mkdir nuc_apache_package
pushd nuc_apache_package
tar -czvf nuc_package.tar.gz ../hlc ../middleware
popd
#   3. publish package
rm -f /var/www/html/nuc/nuc_package.tar.gz
cp ./nuc_apache_package/nuc_package.tar.gz /var/www/html/nuc
rm -f /var/www/html/nuc/DDS_DOMAIN
echo $DDS_DOMAIN >/var/www/html/nuc/DDS_DOMAIN
rm -f /var/www/html/nuc/DDS_INITIAL_PEER
echo $DDS_INITIAL_PEER >/var/www/html/nuc/DDS_INITIAL_PEER

sleep infinity