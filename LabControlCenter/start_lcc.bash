cd ~/dev/software/LabControlCenter

# Load environment Variables
export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

./build/LabControlCenter --dds_domain=12 --dds_initial_peer=$DDS_INITIAL_PEER
