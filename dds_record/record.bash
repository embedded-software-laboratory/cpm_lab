#!/bin/bash
cd "$(dirname "$0")"
export SCRIPT_DIR=$(pwd)
export RECORDING_TIMESTAMP=$(date +%Y_%m_%d_%H_%M_%S)



# Create recording config
export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

cat ./rti_recording_config.xml.template \
| sed -e "s|TEMPLATE_DISCOVERY_URL|${DDS_INITIAL_PEER}|g" \
| sed -e "s|TEMPLATE_RECORDING_FILENAME|recording_${RECORDING_TIMESTAMP}_dat|g" \
| sed -e "s|TEMPLATE_DOMAIN_ID|${DDS_DOMAIN}|g" \
>/tmp/rti_recording_config.xml



# Create recoring directory
cd ~
mkdir recording_${RECORDING_TIMESTAMP}
cd recording_${RECORDING_TIMESTAMP}


# Start recording
rtirecord -cfgFile /tmp/rti_recording_config.xml -cfgName cpm_recorder

# The script continues here after the user presses Ctrl+C



# Convert the recording to CSV
echo "Converting to CSV ..."
sleep 1
export LC_NUMERIC="en_US.UTF-8"
rtirecconv -format csv -decodeChar text -decodeOctet hex -time epoch -compact no -filePrefix topic recording_${RECORDING_TIMESTAMP}_dat*