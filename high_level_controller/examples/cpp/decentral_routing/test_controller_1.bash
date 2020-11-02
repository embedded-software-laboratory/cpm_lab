#!/bin/bash

IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

# This has the advantage over run_distributed, that we get the output of the executable
. ~/dev/software/lab_control_center/bash/environment_variables_local.bash;cd /home/dev/dev/software/high_level_controller/examples/cpp/decentral_routing/build/;./decentral_routing --node_id=high_level_controller1 --simulated_time=false --vehicle_ids=1 --dds_domain=${DDS_DOMAIN} --dds_initial_peer=${DDS_INITIAL_PEER}
