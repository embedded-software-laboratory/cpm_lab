//Includes file descriptions for the LCC

/**
 * \file build.bash
 * \brief Build script that builds the LCC and pulls the YAML repo; also creates a launcher link
 * \ingroup lcc_further
 */

/**
 * \file gdb_run.bash
 * \brief Runs the LCC with GDB (insert 'run' after starting this script in the command line that shows up). Allows for better error tracking.
 * \ingroup lcc_further
 */

/**
 * \file run.bash
 * \brief Runs the LCC with a pre-defined number of vehicles set for the vehicle selection in the UI
 * \ingroup lcc_further
 */

/**
 * \file start_lcc.bash
 * \brief Runs the LCC without the pre-defined number of vehicles
 * \ingroup lcc_further
 */

/**
 * \file valgrind_run.bash
 * \brief Memory-check run of the LCC, rudimentary, unstable
 * \ingroup lcc_further
 */

/**
 * \file commonroad_profiles.yaml
 * \brief YAML file that stores commonroad information for translation/rotation/... for different profiles
 * \ingroup lcc_further
 */

/**
 * \file parameters.yaml
 * \brief Parameters set and stored in the lcc, that are distributed by the LCC 
 * within the network as described in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Parameter+Service and can be edited in the LCC's UI
 * \ingroup lcc_further
 */

/**
 * \file commonroad_file_chooser.config
 * \brief Config file that remembers the last chosen commonroad file
 * \ingroup lcc_further
 */

/**
 * \file file_dialog_open_config.config
 * \brief Config file that remembers the last chosen non-commonroad file
 * \ingroup lcc_further
 */

/////////////////////////////////////////////////////////////////////////////////////
// BASH FILES
/**
 * \defgroup lcc_bash LCC Bash Files
 * \brief LCC file descriptions for files in ./bash (for local & remote deployment & vehicle reboot)
 * \ingroup lcc_further
 */

/**
 * \file copy_to_remote.bash
 * \brief Copies the chosen script to the HLC guest user, starts the script using tmux; 
 * path + script + middleware command line information are transferred as well;
 * does not check if the HLC is reachable beforehand, this is handled internally by the C++ code that calls this script via timeouts
 * \ingroup lcc_bash
 */

/**
 * \file environment_variables_local.bash
 * \brief Sets required environment variables for tmux sessions running locally
 * \ingroup lcc_bash
 */

/**
 * \file environment_variables.bash
 * \brief Sets required environment variables for tmux sessions running on the HLC
 * \ingroup lcc_bash
 */

/**
 * \file reboot_raspberry.bash
 * \brief Sends a reboot signal to the vehicle
 * \ingroup lcc_bash
 */

/**
 * \file remote_kill.bash
 * \brief Kills the middleware and script tmux sessions on the HLC guest, if it is reachable
 * \ingroup lcc_bash
 */

/**
 * \file remote_start.bash
 * \brief Starts middleware and script on the HLC guest via tmux
 * \ingroup lcc_bash
 */

/**
 * \file tmux_middleware.bash
 * \brief Script to start the middleware on the HLC, called by remote_start internally
 * \ingroup lcc_bash
 */

/**
 * \file tmux_script.bash
 * \brief Script to start the script on the HLC, called by remote_start internally
 * \ingroup lcc_bash
 */
////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
// UI FILES
/**
 * \defgroup lcc_ui_files LCC UI Files
 * \brief Short description for the .glade UI files
 * \ingroup lcc_further
 */

/**
 * \file ./ui/master_layout.glade
 * \brief Defines the layout of the LCC program, contains all other UI elements
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/commonroad/commonroad.glade
 * \brief Layout of the Commonroad Tab
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/commonroad/obstacle_toggle.glade
 * \brief Toggle to turn obstacle simulation with trajectory on or off
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/file_chooser/FileChooserDialog.glade
 * \brief Layout for a file chooser dialog
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/file_chooser/FileSaverDialog.glade
 * \brief Layout for a file saver dialog
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/lcc_errors/lcc_errors.glade
 * \brief Layout for the tab that displays internal LCC errors or warnings
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/logger/logger.glade
 * \brief Layout for the tab that shows log messages from the whole network, depending on the log level
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/manual_control/manual_control_ui2.glade
 * \brief Layout for the tab to manually control a vehicle
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/monitoring/monitoring_ui.glade
 * \brief Layout for the monitoring tab on the bottom, that includes vehicle and HLC information including RTT, and HLC reboot options
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/params/params.glade
 * \brief Layout for the Tab where parameters can be seen and set
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/params/params_create.glade
 * \brief Layout for a parameter creation or edit window
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/params/params_show.glade
 * \brief Layout for a window to inspect param information
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/right_tabs/right_tabs.glade
 * \brief Layout for all the UI elements shown in tabs on the right side
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/setup/setup.glade
 * \brief Layout for the setup tab, where the script can be chosen, vehicles can be selected, the simulation can be started or stopped
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/setup/upload_window.glade
 * \brief Layout for an upload window, shown after starting the remote deployment, which displays upload information
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/setup/vehicle_toggle.glade
 * \brief Layout for vehicle toggles in the setup tab
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/timer/timer.glade
 * \brief Layout for the timer tab, where timing information can be seen and the timer can be (re)started and stopped
 * \ingroup lcc_ui_files
 */

/**
 * \file ./ui/style.css
 * \brief Some custom CSS definitions for the LCC's UI
 * \ingroup lcc_ui_files
 */
////////////////////////////////////////////////////////////////////////////////////