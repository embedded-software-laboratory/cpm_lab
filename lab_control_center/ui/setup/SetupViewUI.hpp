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
#include "ui/file_chooser/FileChooserUI.hpp"
#include "ui/timer/TimerViewUI.hpp"
#include "ui/setup/Deploy.hpp"
#include "ui/setup/VehicleToggle.hpp"
#include "ui/setup/UploadWindow.hpp"

#ifndef SIMULATION
    #include "labcam/LabCamIface.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <array>
#include <cstdio> //For popen
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
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

    // Interface to LabCam
#ifndef SIMULATION
    LabCamIface* labcam;
#endif

    //Class to send automated vehicle commands to a list of vehicles, like stop signals after kill has been called
    std::shared_ptr<VehicleAutomatedControl> vehicle_control;

    //Class to simulate obstacles based on the loaded commonroad file
    std::shared_ptr<ObstacleSimulationManager> obstacle_simulation_manager;

    //Function to get a list of all currently online HLCs
    std::function<std::vector<uint8_t>()> get_hlc_ids;

    //Function to get IDs of real vehicles (and simulated ones) which are currently active
    std::function<VehicleData()> get_vehicle_data;
    std::vector<unsigned int> active_real_vehicles;
    std::mutex active_real_vehicles_mutex;
    std::thread check_real_vehicle_data_thread;
    std::atomic_bool is_deployed;
    std::atomic_bool vehicle_data_thread_running;

    //Functions to reset all UI elements after a simulation was performed / before a new one is started
    std::function<void(bool, bool)> reset_timer;
    std::function<void()> reset_time_series_aggregator;
    std::function<void()> reset_obstacle_aggregator;
    std::function<void()> reset_trajectories;
    std::function<void()> reset_vehicle_view;
    std::function<void()> reset_visualization_commands;
    std::function<void()> reset_logs;
    std::function<void(bool)> set_commonroad_tab_sensitive;

    //Function to get the main window
    std::function<Gtk::Window&()> get_main_window;

    //Loading window while HLC scripts are being updated
    //Also: Upload threads and GUI thread (to keep upload work separate from GUI)
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::vector<std::thread> upload_threads; //threads that are responsible for uploading scripts to the HLCs
    std::mutex upload_threads_mutex;
    std::shared_ptr<UploadWindow> upload_window; //window that shows an upload message
    void ui_dispatch(); //dispatcher callback for the UI thread
    /**
     * \brief Notify function that gets called by the upload threads when they have finished their work
     * \param hlc_id ID the thread was responsible for
     * \param upload_success Whether the upload was successful
     */
    void notify_upload_finished(uint8_t hlc_id, bool upload_success);
    void kill_all_threads(); //function to join all threads
    bool check_if_online(uint8_t hlc_id); //Check if the HLC is still online
    std::atomic_uint8_t thread_count; //thread counter, set before thread creation so that, if they finish before the next one is created, still threads are only joined after all upload threads that need to be created have finished their work
    size_t notify_count; //counter for notify_upload_finished; if it does not match thread_count after all threads have called it, print an error message (means that there was a setup mistake made at thread creation)
    std::mutex notify_callback_in_use; //the notify_upload_finished function should only be accessible by one thread at once, thus use this mutex
    std::atomic_bool participants_available; //Used by deploy and ui_dispatch in case the upload fails because no HLC was online or no vehicle was selected
    //Horrible way to log an error message, because the UI cannot be accessed directly - if error_msg.size() > 0, emit just triggers that an error msg is added
    std::mutex error_msg_mutex;
    std::vector<std::string> error_msg;
    std::atomic_bool kill_called; //Must be known to the UI functions - undo grey out of the UI elements after the notification window is closed
    void perform_post_kill_cleanup();

    //IPS switch callback (-> lab mode)
    void switch_ips_set();

    //diagnosis switch callback 
    void switch_diagnosis_set();

    //Overall deploy functions, to deploy / kill script + middleware + vehicle software locally /remotely
    void deploy_applications();
    void kill_deployed_applications();

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

public:
    /**
     * \brief Constructor
     * \param _vehicle_control Allows to send automated commands to the vehicles, like stopping them at their current position after simulation
     * \param _obstacle_simulation_manager Used to simulate obstacles defined in currently loaded commonroad file - reference here for start(), stop()
     * \param _get_hlc_ids Get all IDs of currently active HLCs for correct remote deployment
     * \param _get_vehicle_data Used to get currently active vehicle IDs
     * \param _reset_timer Reset timer & set up a new one for the next simulation
     * \param _reset_time_series_aggregator Reset received vehicle data
     * \param _reset_obstacle_aggregator Reset received obstacle data
     * \param _reset_trajectories Reset received vehicle trajectories / drawing them in the map
     * \param _reset_vehicle_view Reset list of connected vehicles
     * \param _reset_visualization_commands Reset all visualization commands that were sent before
     * \param _reset_logs Reset all logs that were sent before
     * \param _set_commonroad_tab_sensitive Set commonroad loading tab to (un)sensitive to hinder the user from creating invalid states during simulation
     * \param argc Command line argument (from main())
     * \param argv Command line argument (from main())
     */
    SetupViewUI(
        std::shared_ptr<Deploy> deploy_functions, 
        std::shared_ptr<VehicleAutomatedControl> _vehicle_control, 
        std::shared_ptr<ObstacleSimulationManager> _obstacle_simulation_manager,
        std::function<std::vector<uint8_t>()> _get_hlc_ids, 
        std::function<VehicleData()> _get_vehicle_data,
        std::function<void(bool, bool)> _reset_timer,
        std::function<void()> _reset_time_series_aggregator,
        std::function<void()> _reset_obstacle_aggregator,
        std::function<void()> _reset_trajectories,
        std::function<void()> _reset_vehicle_view,
        std::function<void()> _reset_visualization_commands,
        std::function<void()> _reset_logs,
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

    /**
     * \brief As the destructor does not seem to work as desired (it does not kill all remaining programs as desired), its
     * functionality is implemented twice. This function can be called in main when a window close operation is detected, to kill
     * the according programs
     */
    void on_lcc_close();
};