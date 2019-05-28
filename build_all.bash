


# Get cpm lib
pushd ..
if [ ! -d "cpm_base" ]; then
    git clone https://git.rwth-aachen.de/CPM/Project/Lab/cpm_base.git
fi
cd cpm_base
cd cpm_lib
bash build_arm.bash  
bash build.bash
popd



pushd LabControlCenter
bash build.bash
popd



pushd vehicle_raspberry_firmware
bash build.bash
popd


pushd controller_test_loop
bash build.bash
popd



pushd ips2
bash build.bash
popd