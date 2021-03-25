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

#pragma once

#include <atomic>
#include <array>
#include <cassert>
#include <chrono>       //For time measurements (timeout for remote deployment)
#include <cstdio>       //For popen
#include <experimental/filesystem> //Used instead of std::filesystem, because some compilers still seem to be outdated
#include <functional>
#include <fstream>
#include <iomanip>      // put_time
#include <iostream>
#include <map>
#include <memory>
#include <regex>        // to replace file contents
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

//To spawn a process & get its PID
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "cpm/Logging.hpp"
#include "ProgramExecutor.hpp"

/**
 * \brief This class is responsible for managing deployment of HLC and vehicle scripts / programs and other participants that are launched from the LCC
 * \ingroup lcc_ui
 */
class Deploy 
{
public:
    /**
     * \brief Constructor that sets variables for the deploy functions that do not change during execution
     * \param _cmd_domain_id domain ID set in the command line (when starting the LCC)
     * \param _cmd_dds_initial_peer dds initial peer set in the command line (when starting the LCC)
     * \param _stop_vehicle Callback function to make the vehicle stop immediately (given a vehicle ID)
     * \param _program_executor Class object that gives "safer" access to fork etc., to prevent memory leaks etc. that may occur in multi threaded programs
     * \param _absolute_exec_path Path of the executable. Is required to construct paths to other important programs in the software repo
     */
    Deploy(
        unsigned int _cmd_domain_id, 
        std::string _cmd_dds_initial_peer, 
        std::function<void(uint8_t)> _stop_vehicle, 
        std::shared_ptr<ProgramExecutor> _program_executor,
        std::string _absolute_exec_path
    );

    /**
     * \brief Deconstructor required because of reboot threads
     */
    ~Deploy();

    //Specific local deploy functions
    /**
     * \brief Start the middleware and the given Matlab/C++ script (using tmux and a system call); pass parameters: domain id, initial peer, simulated time, vehicle id
     * \param use_simulated_time Whether simulated time or real time shall be used for the lab run
     * \param active_vehicle_ids All vehicle IDs that are in use in this run, no matter if real or simulated
     * \param script_path Path to the script, including the script name (and possible file ending)
     * \param script_params Additional script parameters
     */
    void deploy_local_hlc(bool use_simulated_time, std::vector<unsigned int> active_vehicle_ids, std::string script_path, std::string script_params);

    //Specific local deploy functions with 1 HLC per vehicle ID
    /**
     * \brief Start the middleware once and the given Matlab/C++ script (using tmux and a system call) once for each vehicle ID; pass parameters: domain id, initial peer, simulated time, vehicle id
     * \param use_simulated_time Whether simulated time or real time shall be used for the lab run
     * \param active_vehicle_ids All vehicle IDs that are in use in this run, no matter if real or simulated
     * \param script_path Path to the script, including the script name (and possible file ending)
     * \param script_params Additional script parameters
     */
    void deploy_separate_local_hlcs(bool use_simulated_time, std::vector<unsigned int> active_vehicle_ids, std::string script_path, std::string script_params);

    /**
     * \brief Deploy all vehicles that were set to be simulated locally, set real or simulated time (software is started using tmux)
     * \param simulated_vehicle_ids IDs of vehicles to simulate locally
     * \param use_simulated_time True if simulated time should be used, false if real time should be used
     */
    void deploy_sim_vehicles(std::vector<unsigned int> simulated_vehicle_ids, bool use_simulated_time);
    /**
     * \brief Deploy a single simulated vehicle by passing its ID
     * \param id ID of vehicle to simulate locally
     * \param use_simulated_time True if simulated time should be used, false if real time should be used
     */
    void deploy_sim_vehicle(unsigned int id, bool use_simulated_time);

    /**
     * \brief Make running vehicles brake and stop
     * \param vehicle_ids IDs of vehicles to stop (works for both simulated and real vehicles)
     */
    void stop_vehicles(std::vector<unsigned int> vehicle_ids);

    //Local kill functions: Kill middleware, script and vehicles using their tmux ID 
    /**
     * \brief Kill locally deployed HLC script, used if deploy_local_hlc was used before (only one script was started locally)
     */
    void kill_local_hlc();
    /**
     * \brief Kill locally deployed HLC scripts, used if deploy_separate_local_hlcs was used before (multiple scripts were started locally)
     */
    void kill_separate_local_hlcs();
    /**
     * \brief Kill all simulated vehicles with the given IDs
     * \param simulated_vehicle_ids Vehicle IDs of simulated vehicle programs to kill
     */
    void kill_sim_vehicles(std::vector<unsigned int> simulated_vehicle_ids);
    /**
     * \brief Kill simulated vehicle program with the given ID
     * \param id Vehicle ID
     */
    void kill_sim_vehicle(unsigned int id);

    /**
     * \brief Kill a single real vehicle using a simple via ssh using sudo reboot - does not work for simulated vehicles started without the LCC
     * \param id of the vehicle, to infer its IP address
     * \param timeout_seconds Timeout in case the IP is not reachable
     */
    void reboot_real_vehicle(unsigned int id, unsigned int timeout_seconds);

    /**
     * \brief Reboot all given HLCs, if they are reachable within the timeout
     * \param ids of the HLCs, to infer the IP addresses
     * \param timeout_seconds Timeout in case the IP is not reachable
     */
    void reboot_hlcs(std::vector<uint8_t> ids, unsigned int timeout_seconds);

    //Deploy and kill the IPS (for position tracking of the real vehicles)
    /**
     * \brief Deploy IPS (for position tracking of the real vehicles)
     */
    void deploy_ips();
    /**
     * \brief Kill IPS started with deploy_ips
     */
    void kill_ips();


    /**
     * \brief Deploy the labcam. After executing this command the labcam will immediately start the recording.
     * \param path The path where the video is to be saved.
     * \param file_name The name of the video file.
     */
    void deploy_labcam(std::string path, std::string file_name);

    // Kill the labcam. After executing this command the labcam will immediately stop the recording and save the result.
    void kill_labcam();


    //! For diagnosis of data done in MonitoringUi, is set in SetupViewUI
    bool diagnosis_switch = false; 

    //Deploy and kill the rtirecordingservice
    /**
     * \brief Start the RTI recording service
     */
    void deploy_recording();
    /**
     * \brief Kill the RTI recording service
     */
    void kill_recording();

    //Specific remote deploy functions
    /**
     * \brief Deploy the script + middleware specified in the UI with the given parameters on the HLC with the given ID. 
     * The script is responsible for one or multiple vehicle ids. The script and all data in the same folder gets copied to the HLC and is then called there.
     * \param hlc_id The ID of the HLC on which the software shall be deployed -> used to get the IP address
     * \param vehicle_ids One or multiple vehicle IDs, comma-separated, as string - to be passed to the middleware and the script
     * \param use_simulated_time Whether simulated time or real time shall be used for the lab run
     * \param script_path Path to the script, including the script name (and possible file ending) - MUST BE ABSOLUTE
     * \param script_params Additional script parameters
     * \param timeout_seconds Time to wait until the exection is aborted
     * \return True if the execution did not have to be aborted and no process-related error occured, false otherwise
     */
    bool deploy_remote_hlc(unsigned int hlc_id, std::string vehicle_ids, bool use_simulated_time, std::string script_path, std::string script_params, unsigned int timeout_seconds);
    /**
     * \brief Kill the script + middleware on the given HLC (again determine the IP from the HLC ID)
     * \param hlc_id ID of the HLC on which to kill the programs
     * \param timeout_seconds Timeout in seconds until the kill process should be terminated
     * \return True if the execution (of the bash script) did not have to be aborted and no process-related error occured, false otherwise
     */
    bool kill_remote_hlc(unsigned int hlc_id, unsigned int timeout_seconds);

    /**
     * \brief Used to create the folder software_top_folder_path(value of variable)/name, in which logs of local tmux sessions started here are stored (for debugging purposes)
     * \param folder_name Name of the log folder, default is lcc_script_logs (better change the default if you want to change the folder name)
     */
    void create_log_folder(std::string folder_name = "lcc_script_logs");

    /**
     * \brief Delete logs in the log folder (as in create_log_folder) that are "outdated" (logs of script and middleware)
     * because the simulation was stopped / a new one is started.
     * If the log folder does not yet exist, it is created.
     * \param folder_name Name of the log folder, default is lcc_script_logs (better change the default if you want to change the folder name)
     */
    void delete_old_logs(std::string folder_name = "lcc_script_logs");

    /**
     * \brief Function that can be used to check if all required scripts are still running (checks for existing tmux sessions)
     * \param script_started False if middleware+script are not running / should not be checked
     * \param deploy_remote Set to true if remote deploy of HLC chosen (will later check for hlc and middleware on remote hosts or be outsourced to other func)
     * \param has_local_hlc Special case for deploy remote, in which case existence of a local HLC will be checked 
     * \param lab_mode_on Set to true if lab mode is on (otherwise will not check for IPS)
     * \param check_for_recording Set to true if recording is on and you want crashes to be checked (otherwise will not check for recording)
     * \return Empty array if everything is fine, else: string of the crashed module
     */
    std::vector<std::string> check_for_crashes(bool script_started, bool deploy_remote, bool has_local_hlc, bool lab_mode_on, bool check_for_recording);

private:
    /**
     * \enum PROCESS_STATE
     * \brief Used for process forking, when spawning and managing spawned processes
     */
    enum PROCESS_STATE {DONE, RUNNING, ERROR};

    //! Contains the path to the software folder of the repo, from which paths to all relevant contained programs can be constructed (e.g. to vehicles, IPS etc.)
    std::string software_folder_path;

    //! Path above software folder, for lcc_script_logs folder
    std::string software_top_folder_path;

    //! DDS Domain ID. This value is set once at startup (as command line parameters).
    unsigned int cmd_domain_id; 
    //! DDS Initial Peer. This value is set once at startup (as command line parameters).
    std::string cmd_dds_initial_peer;

    //! Callback function to send a vehicle stop signal / control to the specified vehicle
    std::function<void(uint8_t)> stop_vehicle;

    //! Provides safer access to deploying functions (uses a child process that was forked before creation of DDS threads etc.)
    std::shared_ptr<ProgramExecutor> program_executor;
    
    //! In case of remote deployment, some vehicles might not be matched because not enough HLCs are available. The remaining HLCs are simulated on the local machine, their ID is stored here.
    std::vector<unsigned int> deployed_local_hlcs;

    //Helper functions
    /**
     * \brief Get only the path of the given script, and its name name
     * \param in string containing the path and name of the scrip in the form script_path/script_name
     * \param out_path string containing only script_path
     * \param out_name string containing only script_name
     */
    void get_path_name(std::string& in, std::string& out_path, std::string& out_name);

    /**
     * \brief Check if the given tmux session already exists - used to kill left-over sessions
     * \param session_id ID of the tmux session
     * \return True if the session exists, false otherwise
     */
    bool session_exists(std::string session_id);

    /**
     * \brief Kill a tmux session with the given session_id - only if it exists (uses session_exists)
     * \param session_id ID of the tmux session
     */
    void kill_session(std::string session_id);

    /**
     * \brief Convert boolean to string - used for command line parameters (for deployment)
     * \param var Boolean to convert to string
     * \return String version of the boolean (true -> "true", false -> "false")
     */
    std::string bool_to_string(bool var);

    // Session name for recording service
    const std::string recording_session = "dds_record";
    //! Tmux session name for the IPS Pipeline
    const std::string ips_session = "ips_pipeline";
    //! Tmux session name for the IPS Basler LED Detection
    const std::string basler_session = "ips_basler";
    //! Tmux session name for the labcam
    const std::string labcam_session = "labcam";
    //! Tmux session name for the middleware
    const std::string middleware_session = "middleware";
    //! Tmux session name for the HLC
    const std::string hlc_session = "high_level_controller";
    //! Tmux session name for the vehicle (followed by ID)
    const std::string vehicle_session = "vehicle";

    //! Log file name part for remote copy
    const std::string remote_copy_log_name = "remote_copy_command";
    //! Log file name part for remote kill
    const std::string remote_kill_log_name = "remote_kill_command";

    //To reboot real vehicles
    //! Map to have access to vehicle IDs <-> reboot thread; threads are used to reboot vehicles s.t. the shell commands do not block the UI; they must be joined at some time
    std::map<unsigned int, std::thread> vehicle_reboot_threads;
    //! Mutex for access to vehicle_reboot_threads
    std::mutex vehicle_reboot_threads_mutex;
    //! To find out if a thread has finished execution (no waiting desired), accessed both by main thread and vehicle reboot thread
    std::map<unsigned int, bool> vehicle_reboot_thread_done;
    //! Mutex to access vehicle_reboot_thread_done
    std::mutex vehicle_reboot_done_mutex;
    /**
     * \brief Function to clear already running vehicle reboot threads, 
     * called whenever a new reboot is asked for. Also, all threads are killed on closing the LCC.
     */
    void join_finished_vehicle_reboot_threads();

    //To reboot HLCs
    //! Map to have access to HLC IDs <-> reboot thread; threads are used to reboot HLCs s.t. the shell commands do not block the UI; they must be joined at some time
    std::map<unsigned int, std::thread> hlc_reboot_threads;
    //! Mutex for access to hlc_reboot_threads
    std::mutex hlc_reboot_threads_mutex;
    //! To find out if a thread has finished execution (no waiting desired), accessed both by main thread and HLC reboot thread
    std::map<unsigned int, bool> hlc_reboot_thread_done;
    //! Mutex to access hlc_reboot_thread_done
    std::mutex hlc_reboot_done_mutex;
    /**
     * \brief Function to clear already running HLC reboot threads, 
     * called whenever a new reboot is asked for. Also, all threads are killed on closing the LCC.
     */
    void join_finished_hlc_reboot_threads();
};
