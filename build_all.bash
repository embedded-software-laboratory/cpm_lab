#!/bin/bash
# exit when any command fails
set -e
# Get directory of bash script
BASH_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${BASH_DIR}
#Get command line arguments
SIMULATION=0
HEADLESS=0

for arg in "$@"
do
    case $arg in
        -s|--simulation)
        SIMULATION=1
	    echo "Building in simulation mode: no IPS, no ARM builds."
        shift # Remove --simulation from processing
        ;;
        --headless)
        HEADLESS=1
	    echo "Building in headless mode: no LCC."
        shift # Remove --headless from processing
        ;;
        *)
        shift # Remove generic argument from processing
        ;;
    esac
done

#make simulation variable available in all bash scripts called
export SIMULATION

# cpm lib
pushd cpm_lib
    if [ $SIMULATION == 0 ]; then
        bash build_arm.bash  
    fi
    bash build.bash
popd

# lcc
if [ $HEADLESS == 0 ]; then
    pushd lab_control_center
        bash build.bash
    popd
fi

pushd high_level_controller
    if [ $SIMULATION == 0 ]; then
        pushd autostart
            bash build.bash
            bash create_nuc_package.bash
        popd
    fi
    pushd examples/cpp/central_routing
        bash build.bash
    popd
    pushd examples/cpp/basic_circle
        bash build.bash
    popd
    pushd examples/cpp/basic_line
        bash build.bash
    popd
    pushd examples/cpp/diagonal_figure_eight
        bash build.bash
    popd
    pushd examples/cpp/two_vehicles_drive
        bash build.bash
    popd
popd

pushd middleware
    bash build.bash
popd

pushd mid_level_controller
    if [ $SIMULATION == 0 ]; then
        bash build.bash
    else
        bash build.bash --simulation
    fi
popd

if [ $SIMULATION == 0 ]; then
    pushd indoor_positioning_system
        bash build.bash
    popd
fi

