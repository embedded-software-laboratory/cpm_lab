# Generates evaluation logs for the specified modes for N (as set) vehicles.
# It places them in the folder ./data/MODE/N/
# It currently uses the planning horizon which is hardcoded in the HLC. To generate data for different planning horizons the value of  N_STEPS_SPEED_PROFILE has to be changed in VehicleTrajectoryPlanningState.hpp.

HORIZON="200"
SEEDSTART=101
SEEDEND=110

for ((SEED=SEEDSTART;SEED<=SEEDEND;SEED++))
do
    mkdir data_seed_${SEED}
    cd ./data_seed_${SEED}
    mkdir -p 200 # horizon in timesteps 
    cd 200
    for MODE in  0 1 2
    do
        mkdir -p $MODE
        cd $MODE
        for N in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
        do
            mkdir $N
            cd ./$N
            ../../../../build/simulation --n=$N --hlc_mode=$MODE --path_seed=$SEED --prio_seed=$SEED >> /dev/null
            cd ..
        done
        cd ..
    done
    cd ..
    cd ..
done