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

//File descriptions for the mid level controller, e.g. for bash files

/**
* \page vehicle_files_page Vehicle Files
* \subpage v_f_start <br>
* \subpage v_f_bootloader <br>
* \subpage v_f_build <br>
* \subpage v_f_deploy <br>
* \subpage v_f_login <br>
* \subpage v_f_monitor <br>
* \subpage v_f_reboot_all <br>
* \subpage v_f_reboot <br>
* \subpage v_f_run_fl <br>
* \subpage v_f_run <br>
* \subpage v_f_readme <br>
* \ingroup vehicle_files
*/

/**
 * \page v_f_start ./package/start.bash
 * \brief Start script on the vehicle, which gets the vehicle ID from the vehicle's IP and with that information starts the vehicle software
 */

/**
 * \page v_f_bootloader bootloader_raspberry.bash
 * \brief This script is installed on each raspberry under "/root/". It downloads the raspberry software from the master PC and runs it using ./package/start.bash
 */

/**
 * \page v_f_build build.bash
 * \brief Builds the vehicle software for the raspberry and for local simulation, publishes it s.t. it can be downloaded by the raspberry
 */

/**
 * \page v_f_deploy deploy.bash
 * \brief Deprecated script to deploy vehicle software on the vehicle / raspberry from the master PC
 */

/**
 * \page v_f_login login_raspberry.bash
 * \brief Deprecated script to log into the vehicle / raspberry (password outdated)
 */

/**
 * \page v_f_monitor monitor_log_raspberry.bash
 * \brief Deprecated script to get logs from the vehicle / raspberry (password outdated)
 */

/**
 * \page v_f_reboot_all reboot_all_raspberry.bash
 * \brief Deprecated script to reboot vehicles / raspberry (password outdated)
 */

/**
 * \page v_f_reboot reboot_raspberry.bash
 * \brief Deprecated script to reboot one vehicle (password outdated)
 */

/**
 * \page v_f_run_fl run_w_flexible_domain.bash
 * \brief Run script for the simulated vehicle where the DDS domain can be changed
 */

/**
 * \page v_f_run run.bash
 * \brief Run script for the simulated vehicle
 */

/**
 * \page v_f_readme ./Readme 
 * \brief Instructions on how to run RTI DDS on the vehicle, soon deprecated due to the switch to eProsima
 */