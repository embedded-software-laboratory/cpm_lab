# Generates evaluation logs for the specified modes for N (as set) vehicles.
# It places them in the folder ./data/MODE/N/
# It currently uses the planning horizon which is hardcoded in the HLC. To generate data for different planning horizons the value of  N_STEPS_SPEED_PROFILE has to be changed in VehicleTrajectoryPlanningState.hpp.

mkdir data
cd ./data
for MODE in  0
do
    mkdir $MODE
    cd $MODE
    for N in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18
    do
        mkdir $N
        cd ./$N
        ../../../build/simulation --n=$N --hlc_mode=$MODE >> cout.log
        cd ..
    done
    cd ..
done

