## HLC
The `dynamic_priorities` HLC is an implementation of a priority based non-cooperative networked controller which uses dynamic priorities to increase feasibility.

The HLC is designed to be run distributedly. This means that in the LCC "Deploy Distributed" has to be activated. 

It implements (0) static, (1) random dynamic and (2) future collision assessment based priorities; (3) vertex ordering is WIP. 

The `dynamic_priorities` executable takes the following options:
- `--hlc_mode=` defaults to 2. Sets the mode with which the priorities of the vehicles are determined (0 = static; 1 = random; 2 = fca; 3 = vertex ordering).
- `--vehicle_ids=` defaults to 4. Sets the ID of the HLC that is being started (e.g. usefully in an terminal or for debugging). Only one should be supplied.
- `--middleware_domain=` defaults to 1. Usually should not be used.

## Simulation
The `simulation` program enables users to run the HLC without using the LCC software. Since in it's current state the actual positions of the vehicles are not used in the `dynamic_priorities` HLC, the simulation results are equal to the results that can be obtained when running the HLC in the LCC.
The starting poses of the vehicles are hardcoded for the _central routing_ map at the beginning of simulation.cpp and are taken from the LCC.
It spawns a thread for each vehicle and calls the respective `plan()` method for the amount of plan steps specified.

The 'simulation' executable takes the following options:
- `--n=`          Sets the number of vehicles that partake in the simulation (max. 20).
- `--hlc_mode=`   Sets the mode with which the priorities of the vehicles are determined (0 = static; 1 = random; 2 = fca; 3 = vertex ordering).
- `--steps=`      Sets the number of time steps the simulation is to be run for. Each time step _dt_ usually takes 400ms (as set in simulation.cpp). 

## Logging
When run each instance of the HLC produces its own csv logfile using `;` as a delimiter. 
It is named `evaluation_i.csv` for each vehicle i and it is structured as follows:

time;speed_profile;fca;new_old_fallback_prio;priority_vector;trajectory;plan_timing
- `time`:                     The timestep in which the log entry was generated.
- `speed_profile`:            The speed for the next 8 minor timesteps. 
- `fca`:                      The future collision assesment value of the vehicle in this timestep (`hlc_mode=2`) or a random value (`hlc_mode=1`) or 0 (`hlc_mode in {0,3}`).
- `new_old_fallback_prio`:    Whether (0) a new priority was assigned and used, (1) the old one was used since the proposed new one was infeasible, (2) runout into save stop, was used.
- `priority_vector`:          The priority distribution the vehicle works (not necessarily the actual priorities; they can differ in regards to the order of the lower prioritised vehicles).
- `trajectory`:               The trajectory for the next timestep (the one that will be send to the vehicle after this plan step).
- `plan_timing`:              The duration the whole `plan()` method took.
Fields that contain multiple values have them separated with a `,`.

## Visualisation
The visualisation python script (`visualisation/visualisation.py`) implements several methods to obtain visualisations from the aforementioned logfiles.

## Bash scripts

- `rtigen.bash`:            Generates the C++ code from the `src/dds_idl` files and places it in `src/dds_idl_cpp`. Is executed as part of `build.bash`.
- `build.bash`:             Builds the repo into `build/` includes _dynamic_priorities_, _simulation_ and _tests_. Requires cmake 3.1 and a C++17 capable compiler.
- `run.bash`:               Example execution of the HLC.
- `run_simulation.bash`:    Example simulation execution.
- `eval.sh`:                Creates a folder with evaluation data. Settings are specified in the file. It currently uses the planning horizon that is hardcoded in the HLC.
