#!/bin/bash

# Argument 1: HLC script path
# Argument 2: HLC script name
# Argument 3: Vehicle ID (also ID of HLC)
# Argument 4: If simulated time should be used
script_path=$1
script_name=$2
vehicle_id=$3
simulated_time=$4

cd /tmp/software/

# Set correct IP in local communication script
my_ip=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
/bin/cp -rf ./middleware/QOS_LOCAL_COMMUNICATION.xml.template ./middleware/build/QOS_LOCAL_COMMUNICATION.xml
sed -i -e "s/TEMPLATE_IP/${my_ip}/g" ./middleware/build/QOS_LOCAL_COMMUNICATION.xml
/bin/cp -rf ./middleware/build/QOS_LOCAL_COMMUNICATION.xml ./high_level_controller/$script_path/

# Copy cpm lib file
/bin/cp ./cpm_lib/build/libcpm.so ./middleware/build/

# Start Middleware and HLC script
# Start screen for middleware; detach and start middleware
tmux new-session -d -s "middleware" "cd /tmp/software/high_level_controller/;bash middleware_start.bash ${vehicle_id} ${simulated_time} &> middleware.txt"

# Start screen for matlab; detach and start matlab
tmux new-session -d -s "high_level_controller" "cd /tmp/software/high_level_controller/;bash hlc_start.bash ${script_path} ${script_name} ${vehicle_id}"