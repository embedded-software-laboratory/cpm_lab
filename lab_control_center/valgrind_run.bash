# Run the LCC with GDB, so that errors are not missed as easily, especially if they cannot be reproduced

cd ~/dev/software/lab_control_center

# Load environment Variables
export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

valgrind --tool=memcheck --leak-check=yes --num-callers=20 --log-file=vgdump --suppressions=gtk.suppression ./build/lab_control_center --dds_domain=$DDS_DOMAIN --dds_initial_peer=$DDS_INITIAL_PEER --number_of_vehicles=20