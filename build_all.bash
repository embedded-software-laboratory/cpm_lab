#!/bin/bash
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

# cpm lib
pushd cpm_lib
    if [ $simulation == 0 ]
    then
        bash build_arm.bash  
    fi
    bash build.bash
popd


pushd lab_control_center
    bash build.bash
popd

pushd high_level_controller
    pushd autostart
        bash build.bash
    popd
    pushd matlab
        bash create_nuc_package.bash
    popd
    pushd examples/cpp/central_routing
        bash build.bash
    popd
popd


pushd middleware
    bash build.bash
popd

pushd mid_level_controller
    if [ $simulation == 0 ]
    then
        bash build.bash
    else
        bash build.bash --simulation
    fi
popd


if [ $simulation == 0 ]
then
    pushd indoor_positioning_system
        bash build.bash
    popd
fi

