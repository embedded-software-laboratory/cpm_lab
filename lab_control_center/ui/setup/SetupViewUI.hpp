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

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "TimeSeriesAggregator.hpp"
#include "ObstacleSimulationManager.hpp"
#include "VehicleManualControl.hpp"
#include "VehicleAutomatedControl.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/RTTTool.hpp"
#include "cpm/get_time_ns.hpp"
#include "ui/file_chooser/FileChooserUI.hpp"
#include "ui/timer/TimerViewUI.hpp"
#include "ui/setup/CrashChecker.hpp"
#include "ui/setup/Deploy.hpp"
#include "ui/setup/Upload.hpp"
#include "ui/setup/UploadWindow.hpp"
#include "ui/setup/VehicleToggle.hpp"
#include "src/GoToPlanner.hpp"

#ifndef SIMULATION
    #include "labcam/LabCamIface.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <array>
#include <cstdio> //For popen
#include <experimental/filesystem> //Used instead of std::filesystem, because some compilers still seem to be outdated
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <vector>

/**
 * \brief This UI class is responsible for the Setup Tab in the LCC
 * It is used to 
 * 1) Set the variables required to start a local / distributed lab run, e.g. selection of a script + parameters, usage of the IPS in lab mode etc
 * 2) Run the selected script + vehicles (real and simulated possible) either on the current machine or on the lab's HLCs
 * 3) Reboot real vehicles
 * \ingroup lcc_ui
 */
class SetupViewUI
{
private:
    //! GTK UI Builder
    Glib::RefPtr<Gtk::Builder> builder;

    //! Parent window of the setup view, allows scrolling and integration in other UI elements
    Gtk::ScrolledWindow* parent = nullptr;
    //! Contains all other UI elements
    Gtk::Widget* setup_box = nullptr;

    //Script and script parameters
    //! Entry to set a script path manually
    Gtk::Entry* script_path = nullptr;
    //! To specify command line parameters for the script
    Gtk::Entry* script_params = nullptr;
    //! Button to set a script via a file chooser
    Gtk::Button* button_choose_script = nullptr;
    
    //Vehicle selection
    //! Button to simulate no vehicle
    Gtk::Button* button_select_none = nullptr;
    //! Button to simulate all vehicles (real vehicles stay real and are not replaced)
    Gtk::Button* button_select_all_simulated = nullptr;

    //! Switch to set timer (simulated or real time)
    Gtk::Switch* switch_simulated_time = nullptr;
    //! Switch to decide if recordings with the labcam should be done
    Gtk::Switch* switch_record_labcam = nullptr;

    //! Switch to (de)activate IPS
    Gtk::Switch* switch_lab_mode = nullptr;

    //! Switch to (de)activate diagnosis
    Gtk::Switch* switch_diagnosis = nullptr;

    //! Switch to (de)activate distributed / remote deployment on HLCs (NUCs) and locally
    Gtk::Switch* switch_deploy_distributed = nullptr;

    //! Button to start simulation
    Gtk::Button* button_deploy = nullptr;
    //! Button to stop simulation
    Gtk::Button* button_kill = nullptr;
    //! Button to go to formation
    Gtk::Button* button_go_to_formation = nullptr;

    //! Box that shows vehicle toggles - these can be used to simulate vehicles, stop simulation or reboot real vehicles
    Gtk::FlowBox* vehicle_flowbox = nullptr;
    //! Position in the vector and vehicle ID correlate (ID-1 = position)
    std::vector<std::shared_ptr<VehicleToggle>> vehicle_toggles; 
    //! For ui thread (UI changes should only be done there), to know that vehicle toggle states must be updated (because real vehicles have been added / removed)
    std::atomic_bool update_vehicle_toggles;
    //! Reboot timeout for vehicles in seconds (if the reboot cmd does not work within 5 seconds, it is aborted)
    const unsigned int reboot_timeout = 5; 

    /**
     * \brief Callback for vehicle toggles, which is called when the toggle state changes. 
     * 
     * Simulated vehicles are created when the toggle is set to "simulated".
     * Simulated vehicles are killed when the toggle is set to "off".
     * Real vehicles are rebooted if the toggle is set to "real".
     * 
     * \param vehicle_id ID of the vehicle which is associated with the toggle
     * \param state New toggle state
     */
    void vehicle_toggle_callback(unsigned int vehicle_id, VehicleToggle::ToggleState state);
    
    /**
     * \brief Timer function - replace current timer in the whole system when user switches between simulated and real time
     */
    void switch_timer_set();

    //! Class containing all functions that are relevant for deployment, local and remote
    std::shared_ptr<Deploy> deploy_functions;
    //! Wait up to 30s until distributed / remote deployment is aborted if the remote upload was not finished until then (for each thread)
    unsigned int remote_deploy_timeout = 30;
    //! Wait up to 2 seconds until a kill command is aborted if it was not finished until then
    unsigned int remote_kill_timeout = 2;

    //! Used for kill_deployed_applications, so that it cannot be called more than once at a time
    std::mutex kill_button_mutex;
    //! To remember last kill button press - s.t. it can't be spammed (would not be beneficial / can lead to crashes)
    std::thread kill_grey_out_thread;
    //! If the kill_grey_out_thread is still running / used to stop the thread early
    std::atomic_bool kill_grey_out_running;
    //! Tells the UI thread to make the kill button sensitive again if true
    std::atomic_bool undo_kill_grey_out;

#ifndef SIMULATION
    //! Interface to LabCam
    LabCamIface* labcam;
#endif

    //! Class to send automated vehicle commands to a list of vehicles, like stop signals after kill has been called
    std::shared_ptr<VehicleAutomatedControl> vehicle_control;

    //! Class to get a list of all currently online HLCs and if script / middleware are running on them
    std::shared_ptr<HLCReadyAggregator> hlc_ready_aggregator;

    //! Function to get IDs of real vehicles (and simulated ones) which are currently active
    std::function<VehicleData()> get_vehicle_data;
    //! List of currently online real (not simulated) vehicles
    std::vector<unsigned int> active_real_vehicles;
    //! Mutex to access active_real_vehicles
    std::mutex active_real_vehicles_mutex;
    /**
     * \brief Check, using get_vehicle_data, if data from a real vehicle was received within the last 500ms for each ID. 
     * If so, this vehicle ID is considered to be part of a "real" vehicle.
     * If a simulated vehicle with the same ID is currently running, it is killed.
     * Updates active_real_vehicles with the currently active real vehicles.
     * The thread does not operate during simulation, because then the vehicle configuration is assumed to be fixed.
     */
    std::thread check_real_vehicle_data_thread;
    //! The check_real_vehicle_data_thread thread only operates if this is false. Is true during simulation.
    std::atomic_bool is_deployed;
    //! Used as a stop condition for check_real_vehicle_data_thread
    std::atomic_bool vehicle_data_thread_running;

    //Functions to reset all UI elements after a simulation was performed / before a new one is started
    /**
     * \brief Callback function to reset the timer after a simulation was stopped, 
     * before a new one was started and in case of a switch between real and simulated time.
     * Params: Simulated time (false if real time), send stop signal (false if none should be sent)
     */
    std::function<void(bool, bool)> reset_timer;
    //! Already called in on_simulation_start and stop, but also required if a simulated vehicle is turned off, to remove it from monitoring ui
    std::function<void()> reset_vehicle_view;
    //! Callback function that is called when a simulation is started
    std::function<void()> on_simulation_start;
    //! Callback function that is called when a simulation is stopped
    std::function<void()> on_simulation_stop;
    //! Callback function to disable / enable the commonroad tab
    std::function<void(bool)> set_commonroad_tab_sensitive;

    //! Function to update active vehicles on the parameter server
    std::function<void(std::vector<int32_t>)> update_vehicle_ids_parameter;

    //! Callback function to get the main window, which is required for opening the file explorer (needs a parent window)
    std::function<Gtk::Window&()> get_main_window;

    //! To communicate between thread and GUI
    Glib::Dispatcher ui_dispatcher;
    //! Dispatcher callback for the UI thread
    void ui_dispatch(); 

    //! True if in distributed / remote deployment there were not sufficient HLCs for the selected vehicles. In that case, some scripts are deployed locally so that all vehicles have a running script.
    std::atomic_bool both_local_and_remote_deploy;

    //! For loading window while HLC scripts are being updated. Also: Upload threads and GUI thread (to keep upload work separate from GUI).
    std::shared_ptr<Upload> upload_manager;
    //! Watcher thread that checks if the deployed programs still run - else, an error message is displayed
    std::shared_ptr<CrashChecker> crash_checker;

    /**
     * \brief Resets the timer, calls on_simulation_stop callback, enables UI interaction again
     */
    void perform_post_kill_cleanup();

    /**
     * \brief IPS switch callback (-> lab mode)
     */
    void switch_ips_set();

    /**
     * \brief Diagnosis switch callback 
     */
    void switch_diagnosis_set();

    /**
     * \brief Deploy function, to deploy / kill script + middleware + vehicle software locally /remotely
     */
    void deploy_applications();
    
    //! If true, just try to kill processes locally - doing this remotely is not a good idea if the program is being shut down, because there is not time for that.
    std::atomic_bool lcc_closed;

    /**
     * \brief Helper function: Get all currently online vehicle IDs, both for simulated and for real ones
     */
    std::vector<unsigned int> get_vehicle_ids_active();
    /**
     * \brief Helper function: Get all currently online real vehicle IDs
     */
    std::vector<unsigned int> get_vehicle_ids_real();
    /**
     * \brief Helper function: Get all currently online simulated vehicle IDs
     */
    std::vector<unsigned int> get_vehicle_ids_simulated();

    /**
     * \brief UI function - set sensitivity of the UI after pressing deploy (grey out fields in the Setup tab)
     * \param is_sensitive If true, make the UI interactable, else grey it out
     */
    void set_sensitive(bool is_sensitive);

    //! If simulated time should be used. Can be set in the command line (upon starting the LCC).
    bool cmd_simulated_time;

    /**
     * \brief Open file explorer to select script(s) + location
     */
    void open_file_explorer();
    /**
     * \brief Callback called by file explorer after file selection / closing without selecting a file.
     * \param file_string Path of the selected file
     * \param has_file True if a file was selected, else file_string should be ignored and the currently selected file is not changed
     */
    void file_explorer_callback(std::string file_string, bool has_file);
    //! Reference to the file chooser object / window
    std::shared_ptr<FileChooserUI> file_chooser_window;

    /**
     * \brief Vehicle button callback to set all vehicles in the toggle list that are not real to simulated
     */
    void select_all_vehicles_sim();
    /**
     * \brief Vehicle button callback to kill all simulated vehicles
     */
    void select_no_vehicles();

    //! To remember if a simulation is currently running, relevant e.g. for the kill button
    std::atomic_bool simulation_running;

    //! Mutex for access to vehicle_to_hlc_map
    std::mutex vehicle_to_hlc_mutex;
    //! For vehicle<->HLC mapping - which HLC (ID, uint8_t) simulates the script for which vehicle (ID, uint32_t)
    std::map<uint32_t, uint8_t> vehicle_to_hlc_map;

    //! Calls the function to go to formation
    void run_go_to_formation();

public:

    /**
     * \brief To kill all deployed scripts / middleware / ... locally and remotely
     */
    void kill_deployed_applications();

    /**
     * \brief Constructor
     * \param _deploy_functions Manages all deploy technicalities, like creating tmux sessions, calling bash scripts etc
     * \param _vehicle_control Allows to send automated commands to the vehicles, like stopping them at their current position after simulation
     * \param _hlc_ready_aggregator Get all IDs of currently active HLCs for correct remote deployment, get currently running scripts etc
     * \param _get_vehicle_data Used to get currently active vehicle IDs
     * \param _reset_timer Reset timer & set up a new one for the next simulation
     * \param _reset_vehicle_view Reset shown data for vehicles in monitoring ui, called when a simulated vehicle is turned off
     * \param _on_simulation_start Callback that can be registered in e.g. main to perform changes on other modules when the simulation starts
     * \param _on_simulation_stop Callback that can be registered in e.g. main to perform changes on other modules when the simulation stops
     * \param _set_commonroad_tab_sensitive Set commonroad loading tab to (un)sensitive to hinder the user from creating invalid states during simulation
     * \param _absolute_exec_path Path of the executable. Is required to construct default path for HLC script selection
     * \param argc Command line argument (from main())
     * \param argv Command line argument (from main())
     */
    SetupViewUI(
        std::shared_ptr<Deploy> _deploy_functions, 
        std::shared_ptr<VehicleAutomatedControl> _vehicle_control, 
        std::shared_ptr<HLCReadyAggregator> _hlc_ready_aggregator, 
        std::function<VehicleData()> _get_vehicle_data,
        std::function<void(bool, bool)> _reset_timer,
        std::function<void()> _reset_vehicle_view,
        std::function<void()> _on_simulation_start,
        std::function<void()> _on_simulation_stop,
        std::function<void(bool)> _set_commonroad_tab_sensitive,
        std::function<void(std::vector<int32_t>)> _update_vehicle_ids_parameter,
        std::string _absolute_exec_path,
        unsigned int argc, 
        char *argv[]
        );
    ~SetupViewUI();

    /**
     * \brief Set the callback function that returns a reference to the application's main window
     * \param _get_main_window Returns reference to the main window, which is "needed" for window creation (works without it, but not without Gtk complaining in the terminal)
     */
    void set_main_window_callback(std::function<Gtk::Window&()> _get_main_window);

    /**
     * \brief Returns true if the given ID corresponds to an online real vehicle, else false
     * \param vehicle_id The ID for which to determine if the corresponding vehicle is online and real
     */
    bool is_active_real(unsigned int vehicle_id);

    /**
     * \brief Get the parent widget of SetupViewUI to put it in a parent container
     */
    Gtk::Widget* get_parent();

    /**
     * \brief This might be subject to change.
     * Returns: True if a simulation is running, and in that case a map with mappings from vehicle ID to HLC ID.
     * (During distributed / remote deployment, each vehicle gets commands from its own HLC. Thus, each vehicle ID is associated to an HLC ID.)
     */
    std::pair<bool, std::map<uint32_t, uint8_t>> get_vehicle_to_hlc_matching();

    /**
     * \brief As the destructor does not seem to work as desired (it does not kill all remaining programs as desired), its
     * functionality is implemented twice. This function can be called in main when a window close operation is detected, to kill
     * all programs currently running due to this object (except remotely running programs - closing them can take a while,
     * and thus too long for reacting to a LCC close event)
     */
    void on_lcc_close();

    /**
     * \brief Returns the crash checker object, which must be created in the SetupViewUI object because
     * it internally relies on upload_manager, which in turn has direct access to UI elements of the setup object
     * and is created in its constructor.
     */
    std::shared_ptr<CrashChecker> get_crash_checker();
};
