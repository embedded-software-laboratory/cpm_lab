//File descriptions for the mid level controller, e.g. for bash files

/**
 * \file ./package/start.bash
 * \brief Start script on the vehicle, which gets the vehicle ID from the vehicle's IP and with that information starts the vehicle software
 * \ingroup vehicle_files
 */

/**
 * \file bootloader_raspberry.bash
 * \brief This script is installed on each raspberry under "/root/". It downloads the raspberry software from the master PC and runs it using ./package/start.bash
 * \ingroup vehicle_files
 */

/**
 * \file build.bash
 * \brief Builds the vehicle software for the raspberry and for local simulation, publishes it s.t. it can be downloaded by the raspberry
 * \ingroup vehicle_files
 */

/**
 * \file deploy.bash
 * \brief Deprecated script to deploy vehicle software on the vehicle / raspberry from the master PC
 * \ingroup vehicle_files
 */

/**
 * \file login_raspberry.bash
 * \brief Deprecated script to log into the vehicle / raspberry (password outdated)
 * \ingroup vehicle_files
 */

/**
 * \file monitor_log_raspberry.bash
 * \brief Deprecated script to get logs from the vehicle / raspberry (password outdated)
 * \ingroup vehicle_files
 */

/**
 * \file reboot_all_raspberry.bash
 * \brief Deprecated script to reboot vehicles / raspberry (password outdated)
 * \ingroup vehicle_files
 */

/**
 * \file reboot_raspberry.bash
 * \brief Deprecated script to reboot one vehicle (password outdated)
 * \ingroup vehicle_files
 */

/**
 * \file run_w_flexible_domain.bash
 * \brief Run script for the simulated vehicle where the DDS domain can be changed
 * \ingroup vehicle_files
 */

/**
 * \file run.bash
 * \brief Run script for the simulated vehicle
 * \ingroup vehicle_files
 */

/**
 * \dir ./Readme 
 * \brief Instructions on how to run RTI DDS on the vehicle, soon deprecated due to the switch to eProsima
 * \ingroup vehicle_files
 */