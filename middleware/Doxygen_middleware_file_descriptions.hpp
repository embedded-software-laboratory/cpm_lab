//File descriptions for middleware files

/**
* \page mw_files Middleware Files
* \subpage mw_build <br>
* \subpage mw_run <br>
* \subpage qos_loc <br>
* \ingroup middleware_files
*/

/**
 * \page mw_build build.bash
 * \brief Build script for the middleware
 * 
 * Also creates a package to download for the NUC/HLC when it boots to get the latest version of the middleware.
 */

/**
 * \page mw_run run.bash
 * \brief Run script for the middleware.
 * 
 * Works given a vehicle ID (or multiple IDs, comma-separated) and a parameter that determines if simulated time should be used (true / false).
 */

/**
* \page qos_loc QOS_LOCAL_COMMUNICATION.xml.template
* \brief Contains QoS settings
* 
* QoS settings for the DDS participant used for the local-only communication between middleware and HLC. 
* See https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Middleware+Usage
*/