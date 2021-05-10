// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

//Includes file descriptions for the LCC

/**
 * \defgroup lcc_further_files LCC Files
 * \brief Files for the LCC, e.g. build.bash
 * \ingroup lcc_further
 */

/**
* \page lcc_further_page LCC Further Files
* \subpage lcc_f_build <br>
* \subpage lcc_f_gdb <br>
* \subpage lcc_f_run <br>
* \subpage lcc_f_start <br>
* \subpage lcc_f_valgrind <br>
* \subpage lcc_f_commonroad <br>
* \subpage lcc_f_parameters <br>
* \subpage lcc_f_commonroad_file <br>
* \subpage lcc_f_file_dialog <br>
* \ingroup lcc_further_files
*/

/**
 * \page lcc_f_build build.bash
 * \brief Build script that builds the LCC and pulls the YAML repo; also creates a launcher link
 */

/**
 * \page lcc_f_gdb gdb_run.bash
 * \brief Runs the LCC with GDB (insert 'run' after starting this script in the command line that shows up). Allows for better error tracking.
 */

/**
 * \page lcc_f_run run.bash
 * \brief Runs the LCC with a pre-defined number of vehicles set for the vehicle selection in the UI
 */

/**
 * \page lcc_f_start start_lcc.bash
 * \brief Runs the LCC without the pre-defined number of vehicles
 */

/**
 * \page lcc_f_valgrind valgrind_run.bash
 * \brief Memory-check run of the LCC, rudimentary, unstable
 */

/**
 * \page lcc_f_commonroad commonroad_profiles.yaml
 * \brief YAML file that stores commonroad information for translation/rotation/... for different profiles
 */

/**
 * \page lcc_f_parameters parameters.yaml
 * \brief Parameters set and stored in the lcc, that are distributed by the LCC 
 * within the network as described in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Parameter+Service and can be edited in the LCC's UI
 */

/**
 * \page lcc_f_commonroad_file commonroad_file_chooser.config
 * \brief Config file that remembers the last chosen commonroad file. Soon outdated and replaced by a YAML file.
 */

/**
 * \page lcc_f_file_dialog file_dialog_open_config.config
 */

/////////////////////////////////////////////////////////////////////////////////////
// BASH FILES
/**
 * \defgroup lcc_bash LCC Bash Files
 * \brief LCC file descriptions for files in ./bash (for local & distributed / remote deployment & vehicle reboot)
 * \ingroup lcc_further
 */

/**
* \page lcc_bash_page LCC Bash Files
* \subpage lcc_bash_copy <br>
* \subpage lcc_bash_env_loc <br>
* \subpage lcc_bash_env <br>
* \subpage lcc_bash_reboot_r <br>
* \subpage lcc_bash_remote_k <br>
* \subpage lcc_bash_remote_s <br>
* \subpage lcc_bash_tmux_mid <br>
* \subpage lcc_bash_tmux_script <br>
* \ingroup lcc_bash
*/

/**
 * \page lcc_bash_copy ./bash/copy_to_remote.bash
 * \brief Copies the chosen script to the HLC guest user, starts the script using tmux; 
 * path + script + middleware command line information are transferred as well;
 * does not check if the HLC is reachable beforehand, this is handled internally by the C++ code that calls this script via timeouts
 */

/**
 * \page lcc_bash_env_loc ./bash/environment_variables_local.bash
 * \brief Sets required environment variables for tmux sessions running locally
 */

/**
 * \page lcc_bash_env ./bash/environment_variables.bash
 * \brief Sets required environment variables for tmux sessions running on the HLC
 */

/**
 * \page lcc_bash_reboot_r ./bash/reboot_raspberry.bash
 * \brief Sends a reboot signal to the vehicle
 */

/**
 * \page lcc_bash_remote_k ./bash/remote_kill.bash
 * \brief Kills the middleware and script tmux sessions on the HLC guest, if it is reachable
 */

/**
 * \page lcc_bash_remote_s ./bash/remote_start.bash
 * \brief Starts middleware and script on the HLC guest via tmux
 */

/**
 * \page lcc_bash_tmux_mid ./bash/tmux_middleware.bash
 * \brief Script to start the middleware on the HLC, called by remote_start internally
 */

/**
 * \page lcc_bash_tmux_script ./bash/tmux_script.bash
 * \brief Script to start the script on the HLC, called by remote_start internally
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
* \page lcc_ui_file_page LCC UI Files
* \subpage lcc_ui_master <br>
* \subpage lcc_ui_commonroad <br>
* \subpage lcc_ui_obstacle_toggle <br>
* \subpage lcc_ui_file_chooser <br>
* \subpage lcc_ui_file_saver <br>
* \subpage lcc_ui_errors <br>
* \subpage lcc_ui_logger <br>
* \subpage lcc_ui_manual_control <br>
* \subpage lcc_ui_monitoring <br>
* \subpage lcc_ui_params <br>
* \subpage lcc_ui_params_create <br>
* \subpage lcc_ui_params_show <br>
* \subpage lcc_ui_right_tabs <br>
* \subpage lcc_ui_setup <br>
* \subpage lcc_ui_upload_window <br>
* \subpage lcc_ui_vehicle_toggle <br>
* \subpage lcc_ui_timer <br>
* \subpage lcc_ui_style <br>
* \ingroup lcc_ui_files
*/

/**
 * \page lcc_ui_master ./ui/master_layout.glade
 * \brief Defines the layout of the LCC program, contains all other UI elements
 */

/**
 * \page lcc_ui_commonroad ./ui/commonroad/commonroad.glade
 * \brief Layout of the Commonroad Tab
 */

/**
 * \page lcc_ui_obstacle_toggle ./ui/commonroad/obstacle_toggle.glade
 * \brief Toggle to turn obstacle simulation with trajectory on or off
 */

/**
 * \page lcc_ui_file_chooser ./ui/file_chooser/FileChooserDialog.glade
 * \brief Layout for a file chooser dialog
 */

/**
 * \page lcc_ui_file_saver ./ui/file_chooser/FileSaverDialog.glade
 * \brief Layout for a file saver dialog
 */

/**
 * \page lcc_ui_errors ./ui/lcc_errors/lcc_errors.glade
 * \brief Layout for the tab that displays internal LCC errors or warnings
 */

/**
 * \page lcc_ui_logger ./ui/logger/logger.glade
 * \brief Layout for the tab that shows log messages from the whole network, depending on the log level
 */

/**
 * \page lcc_ui_manual_control ./ui/manual_control/manual_control_ui2.glade
 * \brief Layout for the tab to manually control a vehicle
 */

/**
 * \page lcc_ui_monitoring ./ui/monitoring/monitoring_ui.glade
 * \brief Layout for the monitoring tab on the bottom, that includes vehicle and HLC information including RTT, and HLC reboot options
 */

/**
 * \page lcc_ui_params ./ui/params/params.glade
 * \brief Layout for the Tab where parameters can be seen and set
 */

/**
 * \page lcc_ui_params_create ./ui/params/params_create.glade
 * \brief Layout for a parameter creation or edit window
 */

/**
 * \page lcc_ui_params_show ./ui/params/params_show.glade
 * \brief Layout for a window to inspect param information
 */

/**
 * \page lcc_ui_right_tabs ./ui/right_tabs/right_tabs.glade
 * \brief Layout for all the UI elements shown in tabs on the right side
 */

/**
 * \page lcc_ui_setup ./ui/setup/setup.glade
 * \brief Layout for the setup tab, where the script can be chosen, vehicles can be selected, the simulation can be started or stopped
 */

/**
 * \page lcc_ui_upload_window ./ui/setup/upload_window.glade
 * \brief Layout for an upload window, shown after starting the distributed / remote deployment, which displays upload information
 */

/**
 * \page lcc_ui_vehicle_toggle ./ui/setup/vehicle_toggle.glade
 * \brief Layout for vehicle toggles in the setup tab
 */

/**
 * \page lcc_ui_timer ./ui/timer/timer.glade
 * \brief Layout for the timer tab, where timing information can be seen and the timer can be (re)started and stopped
 */

/**
 * \page lcc_ui_style ./ui/style.css
 * \brief Some custom CSS definitions for the LCC's UI
 */
////////////////////////////////////////////////////////////////////////////////////