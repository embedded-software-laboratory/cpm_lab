This folder contains the implementation to solve CommonRoad planning problems.
- Call main_vehicle_ids.m from the labcontrolcenter for the simulation
- The folder Map contains the CommonRoad files with the planning problems. The selected file has to be also written into main_vehicle_ids.m ('filepath'), because the LCC currently does not pass the filepath as function argument when calling main_vehicle_ids. The file is read once and then cached in a .mat-file with the same name. if the XML file is updated, the .mat has to be deleted in order to read the XML file with the updates. This reading process of the XML file takes some time (which is why it is cached).
- The folder RTree is needed to build the RTree. The original version of Guttman is used, so there is room for improvements as in R* and R+ Trees for future work.

- The folder ControllerFiles contains all files for the MPC problem. Priority_QP.m is the function to call

- PathPlanner is the path planning class, which uses Dijkstra to find a path from start to goal at the start, and later updates the remaining path by discarding path edges we already passed.

- configVehicle.m is used for the configuration of the HLC/MPC

- vehicle.m is a class which is instanced for every vehicle. This class can be seen as the local HLC of each vehicle.
