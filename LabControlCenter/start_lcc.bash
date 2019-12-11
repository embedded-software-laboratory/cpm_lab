cd ~/dev/software/LabControlCenter

# Load environment Variables
export IP_SELF="$(hostname -I)"
export IP_SELF="$(echo $IP_SELF)"
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

./build/LabControlCenter --dds_domain=$DDS_DOMAIN --dds_initial_peer=$DDS_INITIAL_PEER