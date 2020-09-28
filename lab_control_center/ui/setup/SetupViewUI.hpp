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

#ifndef SIMULATION
    #include "labcam/LabCamIface.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <array>
#include <cstdio> //For popen
#include <filesystem>
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
 */
class SetupViewUI
{
private:
    //Builder and pointer to UI elements
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::ScrolledWindow* parent = nullptr;
    Gtk::Widget* setup_box = nullptr;

    //Script and script parameters
    Gtk::Entry* script_path = nullptr;
    Gtk::Entry* script_params = nullptr;
    Gtk::Button* button_choose_script = nullptr;
    
    //Vehicle selection
    Gtk::Button* button_select_none = nullptr;
    Gtk::Button* button_select_all_simulated = nullptr;

    //Set timer (simulated or real time)
    Gtk::Switch* switch_simulated_time = nullptr;
    Gtk::Switch* switch_record_labcam = nullptr;

    //(De)Activate IPS
    Gtk::Switch* switch_lab_mode = nullptr;

    //(De)Activate diagnosis
    Gtk::Switch* switch_diagnosis = nullptr;

    //(De)Activate remote deployment on HLCs (NUCs)
    Gtk::Switch* switch_deploy_remote = nullptr;

    //Start / stop simulation
    Gtk::Button* button_deploy = nullptr;
    Gtk::Button* button_kill = nullptr;

    //Vehicles - toggles in box to turn them on/off/simulated
    Gtk::FlowBox* vehicle_flowbox = nullptr;
    std::vector<std::shared_ptr<VehicleToggle>> vehicle_toggles; //Position in the vector and vehicle ID correlate (ID-1 = position)
    std::atomic_bool update_vehicle_toggles; //For ui thread
    const unsigned int reboot_timeout = 5; //Reboot time in seconds

    //Remember which vehicle toggles were set to real before
    std::vector<unsigned int> vehicle_toggles_set_to_real;

    //Callback for vehicle toggles - simulated vehicles are now created when the toggle is set to "simulated"
    void vehicle_toggle_callback(unsigned int vehicle_id, VehicleToggle::ToggleState state);
    
    //Timer function - replace current timer in the whole system when user switches between simulated and real time
    std::shared_ptr<TimerViewUI> timer_ui;
    void switch_timer_set();

    //Class containing all functions that are relevant for deployment, local and remote
    std::shared_ptr<Deploy> deploy_functions;
    unsigned int remote_deploy_timeout = 30; //Wait for 30s until the deployment is aborted (for each thread)
    unsigned int remote_kill_timeout = 2; //Wait for 2 seconds until kill is aborted

    //To remember last kill button press - s.t. it can't be spammed (would not be beneficial / can lead to crashes)
    std::mutex kill_button_mutex;
    std::thread kill_grey_out_thread;
    std::atomic_bool kill_grey_out_running;
    std::atomic_bool undo_kill_grey_out;
    const uint64_t kill_timeout = 3e9; //3 seconds

    // Interface to LabCam
#ifndef SIMULATION
    LabCamIface* labcam;
#endif

    //Class to send automated vehicle commands to a list of vehicles, like stop signals after kill has been called
    std::shared_ptr<VehicleAutomatedControl> vehicle_control;

    //Class to get a list of all currently online HLCs and if script / middleware are running on them
    std::shared_ptr<HLCReadyAggregator> hlc_ready_aggregator;

    //Function to get IDs of real vehicles (and simulated ones) which are currently active
    std::function<VehicleData()> get_vehicle_data;
    std::vector<unsigned int> active_real_vehicles;
    std::mutex active_real_vehicles_mutex;
    std::thread check_real_vehicle_data_thread;
    std::atomic_bool is_deployed;
    std::atomic_bool vehicle_data_thread_running;

    //Functions to reset all UI elements after a simulation was performed / before a new one is started
    std::function<void(bool, bool)> reset_timer;
    std::function<void()> on_simulation_start;
    std::function<void()> on_simulation_stop;
    std::function<void(bool)> set_commonroad_tab_sensitive;

    //Function to get the main window
    std::function<Gtk::Window&()> get_main_window;

    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    void ui_dispatch(); //dispatcher callback for the UI thread

    std::atomic_bool both_local_and_remote_deploy; //True if in remote deployment local HLC had to be started for additional vehicles

    //Loading window while HLC scripts are being updated
    //Also: Upload threads and GUI thread (to keep upload work separate from GUI)
    std::shared_ptr<Upload> upload_manager;
    //Watcher thread that checks if the locally deployed programs still run - else, an error message is displayed
    std::shared_ptr<CrashChecker> crash_checker;
    void perform_post_kill_cleanup();

    //IPS switch callback (-> lab mode)
    void switch_ips_set();

    //diagnosis switch callback 
    void switch_diagnosis_set();

    //Overall deploy functions, to deploy / kill script + middleware + vehicle software locally /remotely
    void deploy_applications();
    std::atomic_bool lcc_closed; //If true, just try to kill processes locally - doing this remotely is not a good idea if the program is being shut down

    //Helper functions to get the currently selected vehicle IDs, IDs of real vehicles and IDs of simulated vehicles
    std::vector<unsigned int> get_vehicle_ids_active();
    std::vector<unsigned int> get_vehicle_ids_real();
    std::vector<unsigned int> get_vehicle_ids_simulated();

    //UI function - set sensitivity of the UI after pressing deploy (grey out fields in the Setup tab)
    void set_sensitive(bool is_sensitive);

    //Get parameters that were set in the command line (upon starting the LCC)
    bool cmd_simulated_time;

    //File chooser to select script(s) + location
    void open_file_explorer();
    void file_explorer_callback(std::string file_string, bool has_file);
    std::shared_ptr<FileChooserUI> file_chooser_window;

    //Vehicle button toggle callbacks, to set which vehicles are real / simulated / deactivated
    void select_all_vehicles_sim();
    void select_no_vehicles();

    //For vehicle to HLC mapping
    std::atomic_bool simulation_running;
    std::mutex vehicle_to_hlc_mutex;
    std::map<uint32_t, uint8_t> vehicle_to_hlc_map;

public:
    void kill_deployed_applications();
    /**
     * \brief Constructor
     * \param _deploy_functions Manages all deploy technicalities, like creating tmux sessions, calling bash scripts etc
     * \param _vehicle_control Allows to send automated commands to the vehicles, like stopping them at their current position after simulation
     * \param _hlc_ready_aggregator Get all IDs of currently active HLCs for correct remote deployment, get currently running scripts etc
     * \param _get_vehicle_data Used to get currently active vehicle IDs
     * \param _reset_timer Reset timer & set up a new one for the next simulation
     * \param _on_simulation_start Callback that can be registered in e.g. main to perform changes on other modules when the simulation starts
     * \param _on_simulation_stop Callback that can be registered in e.g. main to perform changes on other modules when the simulation stops
     * \param _set_commonroad_tab_sensitive Set commonroad loading tab to (un)sensitive to hinder the user from creating invalid states during simulation
     * \param argc Command line argument (from main())
     * \param argv Command line argument (from main())
     */
    SetupViewUI(
        std::shared_ptr<Deploy> _deploy_functions, 
        std::shared_ptr<VehicleAutomatedControl> _vehicle_control, 
        std::shared_ptr<HLCReadyAggregator> _hlc_ready_aggregator, 
        std::function<VehicleData()> _get_vehicle_data,
        std::function<void(bool, bool)> _reset_timer,
        std::function<void()> _on_simulation_start,
        std::function<void()> _on_simulation_stop,
        std::function<void(bool)> _set_commonroad_tab_sensitive,
        unsigned int argc, 
        char *argv[]
        );
    ~SetupViewUI();

    /**
     * \brief Set the callback function that returns a reference to the application's main window
     * \param _get_main_window Returns reference to the main window, which is "needed" for window creation (works without it, but not without Gtk complaining in the terminal)
     */
    void set_main_window_callback(std::function<Gtk::Window&()> _get_main_window);

    //True if currently active real vehicle, else false
    bool is_active_real(unsigned int vehicle_id);

    //Get the parent widget to put the view in a parent container
    Gtk::Widget* get_parent();

    //This is subject to change, as the setup ui will be restructured soon
    //Returns: True if a simulation is running and in that case a map with mappings from vehicle ID to HLC ID
    std::pair<bool, std::map<uint32_t, uint8_t>> get_vehicle_to_hlc_matching();
    /**
     * \brief As the destructor does not seem to work as desired (it does not kill all remaining programs as desired), its
     * functionality is implemented twice. This function can be called in main when a window close operation is detected, to kill
     * the according programs
     */
    void on_lcc_close();

    std::shared_ptr<CrashChecker> get_crash_checker();
};