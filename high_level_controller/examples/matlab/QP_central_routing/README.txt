This folder contains the Central Routing Example with CommonRoad data import.
- Call main_vehicle_ids.m from the labcontrolcenter for the simulation
- The folder Map contains the CPM Lab Map as CommonRoad format without a planning problem
The file is read once and then cached in a .mat-file with the same name. if the XML file is updated, the .mat has to be deleted in order to read the XML file with the updates. This reading process of the XML file takes ~30sec (which is why it is cached).
- The folder RTree is needed to build the RTree. The original version of Guttman is used, so there is room for improvements as in R* and R+ Trees for future work.

- The folder ControllerFiles contains all files for the MPC problem. Priority_QP.m is the function to call

- PathPlanner is the path planning class, which dynamically extends the path by choosing random successors
- configVehicle.m is used for the configuration of the HLC/MPC

- vehicle.m is a class which is instanced for every vehicle. This class can be seen as the local HLC of each vehicle.
