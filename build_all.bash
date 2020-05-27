#!/bin/bash
set -e
simulation=0

#Get command line arguments
for arg in "$@"
do
    case $arg in
        -s|--simulation)
        simulation=1
	echo "Building in simulation mode: no IPS, no ARM builds."
        shift # Remove --simulation from processing
        ;;
        *)
        shift # Remove generic argument from processing
        ;;
    esac
done

#make simulation variable available in all bash scripts called
export SIMULATION=$simulation

# Get cpm lib
pushd ..
if [ ! -d "cpm_base" ]; then
     git clone git@git.rwth-aachen.de:CPM/Project/Lab/cpm_base.git
fi

cd cpm_base
cd cpm_lib
if [ $simulation == 0 ]
then
    bash build_arm.bash
fi
bash build.bash
popd


pushd LabControlCenter
bash build.bash
popd

pushd hlc
pushd middleware
bash build.bash
popd
pushd autostart
bash build.bash
popd
pushd matlab
bash create_nuc_package.bash
popd
popd


pushd vehicle_raspberry_firmware
if [ $simulation == 0 ]
then
    bash build.bash
else
    bash build.bash --simulation
fi
popd


pushd controller_test_loop
bash build.bash
popd


pushd central_routing_example
bash build.bash
popd

if [ $simulation == 0 ]
then
    pushd ips2
    bash build.bash
    popd
fi

