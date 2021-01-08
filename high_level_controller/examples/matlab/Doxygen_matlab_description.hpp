// With this file, you can generate a short description for each file in this folder

//////////////////////////////////////////////
/**
 * \file ./init_script.m
 * \brief Init script that loads DDS file definitions, e.g. for the trajectory data type, and creates the fundamental readers and writers
 * Always use this script and always refer to it relative from here or absolute from another place - it will also be placed on the HLC in the same folder
 * 
 * Arguments: Domain ID of Matlab communication (should be 1)
 * Provides: matlabParticipant for the Matlab Domain, stateReader to read vehicle states, 
 *  trajectoryWriter to write vehicle trajectories, systemTriggerReader to get system triggers, 
 *  readyStatusWriter to tell the Middleware that the program is ready to operate, 
 *  trigger_stop to know when systemTriggerReader received a stop signal to stop the program
 * \ingroup matlab
 */

/**
 * \file ./read_system_trigger.m
 * \brief Helper function that looks for and interprets system trigger messages
 * 
 * Arguments: System trigger reader, symbol for stop signals
 * Provides: If the input was a start or stop signal or none of the two
 * \ingroup matlab
 */

/**
 * \file ./QOS_READY_TRIGGER.xml
 * \brief One of the two vital QoS defining XML files for DDS, the other is QOS_LOCAL_COMMUNICATION.xml from the Middleware, see
 * https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Middleware+Usage
 * \ingroup matlab
 */
//////////////////////////////////////////////

//////////////////////////////////////////////
/**
 * \defgroup basic_circle_matlab
 * \ingroup matlab
 */
/**
 * \file ./basic_circle/main.m
 * \brief Matlab version of the basic circle example, see https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Basic+Circle+Example
 * Arguments: Vehicle ID
 * \ingroup basic_circle_matlab
 */
//////////////////////////////////////////////

//////////////////////////////////////////////
/**
 * \defgroup direct_control_matlab
 * \ingroup matlab
 */
/**
 * \file ./direct_control/main.m
 * \brief Matlab example where some vehicleDirect messages are sent to the vehicle
 * Arguments: Vehicle ID
 * \ingroup direct_control_matlab
 */
//////////////////////////////////////////////

//////////////////////////////////////////////
/**
 * \defgroup leader_follower_matlab
 * \ingroup matlab
 */
/**
 * \file ./leader_follower/main.m
 * \brief Matlab example where all remaining vehicles follow one vehicle
 * Arguments: List of vehicle IDs, the vehicles follow the vehicle with the first ID in the list
 * \ingroup leader_follower_matlab
 */
/**
 * \file ./leader_follower/followers.m
 * \brief Function that computes the current trajectory of a follower, if a current state of the leader is given and can thus be followed
 * Arguments: Follower ID, list of vehicle states, leader ID, current time
 * \ingroup leader_follower_matlab
 */
/**
 * \file ./leader_follower/leader_trajectory.m
 * \brief Trajectory definition for the leader to follow, data structure; returns the closest next positions
 * Arguments: Current position
 * \ingroup leader_follower_matlab
 */
/**
 * \file ./leader_follower/leader.m
 * \brief Function that computes the current leader trajectory DDS msg, depending on the current time
 * Arguments: Leader ID, current time
 * \ingroup leader_follower_matlab
 */
//////////////////////////////////////////////

//////////////////////////////////////////////
/**
 * \defgroup platoon_matlab
 * \ingroup matlab
 */
/**
 * \file ./platoon/leader_trajectory.m
 * \brief Trajectory definition for the leader to follow, data structure; returns the closest next positions
 * Arguments: Current position
 * \ingroup platoon_matlab
 */
/**
 * \file ./platoon/leader.m
 * \brief Function that computes the current leader trajectory DDS msg, depending on the current time
 * Arguments: Leader ID, current time
 * \ingroup platoon_matlab
 */
/**
 * \file ./platoon/main_vehicle_amount.m
 * \brief Simple function (name is somewhat misleading) that currently only generates simple trajectories for each vehicle. COLLISIONS ARE POSSIBLE!
 * Arguments: Amount of vehicles n, generates trajectories for vehicles 1 - n
 * \ingroup platoon_matlab
 */
/**
 * \file ./platoon/main_vehicle_ids.m
 * \brief Function that computes the current leader trajectory DDS msg, depending on the current time
 * Arguments: List of vehicles, generates trajectories for those
 * \ingroup platoon_matlab
 */
//////////////////////////////////////////////