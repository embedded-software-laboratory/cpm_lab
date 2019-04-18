


# Get cpm lib
pushd ..
if [ ! -d "cpm_base" ]; then
    git clone https://git.rwth-aachen.de/CPM/Project/Lab/cpm_base.git
    cd cpm_base
    cd cpm_lib
    bash build_arm.bash  
    bash build.bash
fi
popd



pushd LabControlCenter
bash build.bash
popd



pushd vehicle_raspberry_firmware
bash build.bash
popd



pushd ips
bash build.bash
popd