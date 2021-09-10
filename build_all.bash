#!/bin/bash
# exit when any command fails
set -e
# Get directory of bash script
BASH_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${BASH_DIR}

#Get command line arguments
for arg in "$@"
do
    case $arg in
        -s|--simulation)
        export SIMULATION=1
	    echo -e "\e[32m Building in simulation mode: no IPS, no ARM builds. \e[0m"
        shift # Remove --simulation from processing
        ;;
        --headless)
        export HEADLESS=1
	    echo -e "\e[32m Building in headless mode: no LCC. \e[0m"
        shift # Remove --headless from processing
        ;;
        *)
        shift # Remove generic argument from processing
        ;;
    esac
done

# warn users who still use SIMULATION=0
if [ ! -z $SIMULATION ]; then
    if [ $SIMULATION == 0 ]; then
        echo -e "\e[31m WARNING: Deprecated ussage of shell variable SIMULATION detected. \e[0m"
        echo -e "\e[31m          SIMULATION=0 does not work anymore. Please use: \e[0m"
        echo -e "\e[31m          - SIMULATION=1 for simulation only builds \e[0m"
        echo -e "\e[31m          - no SIMULATION variable ('unset SIMULATION') for standard builds \e[0m"
        unset SIMULATION
        echo -e "\e[32m SIMULATION=0 has been unset for you. \e[0m"
    fi
fi

# cpm lib
pushd cpm_lib
    if [ -z $SIMULATION ]; then
        bash build_arm.bash  
    fi
    bash build.bash     # checks for $SIMULATION by itself
popd

# lcc
if [ -z $HEADLESS ]; then
    pushd lab_control_center
        bash build.bash
    popd
fi

pushd high_level_controller
    if [ -z $SIMULATION ]; then
        pushd autostart
            bash build.bash
            bash create_nuc_package.bash
        popd
    fi
    pushd examples/cpp/central_routing
        bash build.bash
    popd
    pushd examples/cpp/distributed_routing
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
    pushd examples/cpp/eight_zero
        bash build.bash
    popd
popd

pushd middleware
    bash build.bash     # checks for $SIMULATION by itself
popd

pushd mid_level_controller
    bash build.bash     # checks for $SIMULATION by itself
popd

if [ -z $SIMULATION ]; then
    pushd indoor_positioning_system
        bash build.bash
    popd
fi
