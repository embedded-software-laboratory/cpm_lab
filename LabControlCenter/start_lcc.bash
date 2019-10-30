cd /home/cpm/dev/software/LabControlCenter
source ../hlc/environment_variables.bash
export DDS_INITIAL_PEER=rtps@udpv4://192.168.1.249:25598

./build/LabControlCenter --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER