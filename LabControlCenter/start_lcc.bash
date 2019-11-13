cd /home/cpm/dev/software/LabControlCenter

# Load environment Variables
. ../hlc/environment_variables.bash

echo $DDS_INITIAL_PEER

./build/LabControlCenter --dds_domain=21 --dds_initial_peer=$DDS_INITIAL_PEER