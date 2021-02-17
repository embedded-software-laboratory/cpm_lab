// With this file, you can generate a short description for each file in this folder; use \page instead of \file - we do not want to parse .m and other file types!

//////////////////////////////////////////////
/**
* \page matlab_page Matlab HLC Files
* \subpage matlab_hlc_init <br>
* \subpage matlab_hlc_read <br>
* \subpage matlab_hlc_qos <br>
* \ingroup matlab
*/

/**
 * \page matlab_hlc_init ./init_script.m
 * \brief Init script that loads DDS file definitions, e.g. for the trajectory data type, and creates the fundamental readers and writers
 * Always use this script and always refer to it relative from here or absolute from another place - it will also be placed on the HLC in the same folder
 * 
 * Arguments: Domain ID of Matlab communication (should be 1)
 * Provides: matlabParticipant for the Matlab Domain, stateReader to read vehicle states, 
 *  trajectoryWriter to write vehicle trajectories, systemTriggerReader to get system triggers, 
 *  readyStatusWriter to tell the Middleware that the program is ready to operate, 
 *  trigger_stop to know when systemTriggerReader received a stop signal to stop the program
 */

/**
 * \page matlab_hlc_read ./read_system_trigger.m
 * \brief Helper function that looks for and interprets system trigger messages
 * 
 * Arguments: System trigger reader, symbol for stop signals
 * Provides: If the input was a start or stop signal or none of the two
 */

/**
 * \page matlab_hlc_qos ./QOS_READY_TRIGGER.xml
 * \brief One of the two vital QoS defining XML files for DDS, the other is QOS_LOCAL_COMMUNICATION.xml from the Middleware, see
 * https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Middleware+Usage
 */
//////////////////////////////////////////////

//////////////////////////////////////////////
/**
 * \defgroup basic_circle_matlab Basic Circle Matlab Example
 * \ingroup matlab
 */
/**
* \page basic_circle_matlab_page Basic Circle Matlab Example
* \subpage basic_circle_main <br>
* \ingroup basic_circle_matlab
*/
/**
 * \page basic_circle_main ./basic_circle/main.m
 * \brief Matlab version of the basic circle example, see https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Basic+Circle+Example
 * Arguments: Vehicle ID
 */
//////////////////////////////////////////////

//////////////////////////////////////////////
/**
 * \defgroup direct_control_matlab Direct Control Matlab Example
 * \ingroup matlab
 */
/**
* \page direct_control_matlab_page Direct Control Matlab Example
* \subpage direct_control_main <br>
* \ingroup direct_control_matlab
*/
/**
 * \page direct_control_main ./direct_control/main.m
 * \brief Matlab example where some vehicleDirect messages are sent to the vehicle
 * Arguments: Vehicle ID
 */
//////////////////////////////////////////////

//////////////////////////////////////////////
/**
 * \defgroup leader_follower_matlab Leader Follower Matlab Example
 * \ingroup matlab
 */
/**
* \page leader_follower_matlab_page Leader Follower Matlab Example
* \subpage leader_follower_main <br>
* \subpage leader_follower_follow <br>
* \subpage leader_follower_lead_tr <br>
* \subpage leader_follower_lead <br>
* \ingroup leader_follower_matlab
*/
/**
 * \page leader_follower_main ./leader_follower/main.m
 * \brief Matlab example where all remaining vehicles follow one vehicle
 * Arguments: List of vehicle IDs, the vehicles follow the vehicle with the first ID in the list
 */
/**
 * \page leader_follower_follow ./leader_follower/followers.m
 * \brief Function that computes the current trajectory of a follower, if a current state of the leader is given and can thus be followed
 * Arguments: Follower ID, list of vehicle states, leader ID, current time
 */
/**
 * \page leader_follower_lead_tr ./leader_follower/leader_trajectory.m
 * \brief Trajectory definition for the leader to follow, data structure; returns the closest next positions
 * Arguments: Current position
 */
/**
 * \page leader_follower_lead ./leader_follower/leader.m
 * \brief Function that computes the current leader trajectory DDS msg, depending on the current time
 * Arguments: Leader ID, current time
 */
//////////////////////////////////////////////

//////////////////////////////////////////////
/**
 * \defgroup platoon_matlab Platoon Matlab Example
 * \ingroup matlab
 */
/**
* \page platoon_matlab_page Platoon Matlab Example
* \subpage platoon_lead_tr <br>
* \subpage platoon_lead <br>
* \subpage platoon_main_v_a <br>
* \subpage platoon_main_v <br>
* \ingroup platoon_matlab
*/
/**
 * \page platoon_lead_tr ./platoon/leader_trajectory.m
 * \brief Trajectory definition for the leader to follow, data structure; returns the closest next positions
 * Arguments: Current position
 */
/**
 * \page platoon_lead ./platoon/leader.m
 * \brief Function that computes the current leader trajectory DDS msg, depending on the current time
 * Arguments: Leader ID, current time
 */
/**
 * \page platoon_main_v_a ./platoon/main_vehicle_amount.m
 * \brief Simple function (name is somewhat misleading) that currently only generates simple trajectories for each vehicle. COLLISIONS ARE POSSIBLE!
 * Arguments: Amount of vehicles n, generates trajectories for vehicles 1 - n
 */
/**
 * \page platoon_main_v ./platoon/main_vehicle_ids.m
 * \brief Function that computes the current leader trajectory DDS msg, depending on the current time
 * Arguments: List of vehicles, generates trajectories for those
 */
//////////////////////////////////////////////