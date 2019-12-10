#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "VehicleManualControl.hpp"
#include "VehicleAutomatedControl.hpp"
#include "cpm/CommandLineReader.hpp"
#include "ui/file_chooser/FileChooserUI.hpp"
#include "ui/timer/TimerViewUI.hpp"
#include "ui/setup/VehicleToggle.hpp"
#include "ui/setup/UploadWindow.hpp"

#include <atomic>
#include <array>
#include <cstdio> //For popen
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

class SetupViewUI
{
private:
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::Widget* parent = nullptr;

    //Script and script parameters
    Gtk::Entry* script_path = nullptr;
    Gtk::Entry* script_params = nullptr;
    Gtk::Button* button_choose_script = nullptr;
    
    //Vehicle selection
    Gtk::Button* button_select_none = nullptr;
    Gtk::Button* button_select_all_simulated = nullptr;
    Gtk::Button* button_select_all_real = nullptr;

    //Set timer (simulated or real time)
    Gtk::Switch* switch_simulated_time = nullptr;

    //(De)Activate IPS
    Gtk::Switch* switch_lab_mode = nullptr;

    //(De)Activate remote deployment on HLCs (NUCs)
    Gtk::Switch* switch_deploy_remote = nullptr;

    //Start / stop simulation
    Gtk::Button* button_deploy = nullptr;
    Gtk::Button* button_kill = nullptr;

    //Vehicles
    Gtk::FlowBox* vehicle_flowbox = nullptr;
    std::vector<std::shared_ptr<VehicleToggle>> vehicle_toggles;

    //Timer function - replace current timer in the whole system when user switches between simulated and real time
    std::shared_ptr<TimerViewUI> timer_ui;
    void switch_timer_set();

    //Class to send automated vehicle commands to a list of vehicles, like stop signals after kill has been called
    std::shared_ptr<VehicleAutomatedControl> vehicle_control;

    //Function to get a list of all currently online HLCs
    std::function<std::vector<uint8_t>()> get_hlc_ids;

    //Loading window while HLC scripts are being updated
    //Also: Upload threads and GUI thread (to keep upload work separate from GUI)
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::vector<std::thread> upload_threads;
    std::shared_ptr<UploadWindow> upload_window;
    void ui_dispatch();
    void notify_upload_finished();
    void kill_all_threads();
    std::atomic_uint8_t thread_count;
    size_t notify_count;
    std::mutex notify_callback_in_use;
    std::atomic_bool upload_success; //Used by deploy and ui_dispatch in case the upload fails because no HLC was online

    //IPS switch callback
    void switch_ips_set();

    //Overall deploy functions
    void deploy_applications();
    void kill_deployed_applications();

    //Specific local deploy functions
    void deploy_hlc_scripts();
    void deploy_middleware();
    void deploy_sim_vehicles();
    void deploy_sim_vehicle(unsigned int id);
    void deploy_ips();
    void kill_ips();

    void kill_hlc_scripts();
    void kill_middleware();
    void kill_vehicles();
    void kill_vehicle(unsigned int id);

    //Specific remote deploy functions
    /**
     * \brief Deploy the script specified in the UI with the given parameters on the HLC with the given ID. The script is responsible for one or multiple vehicle ids
     * \param hlc_id The ID of the HLC
     * \param vehicle_ids One or multiple vehicle IDs, comma-separated
     */
    void deploy_remote_hlc(unsigned int hlc_id, std::string vehicle_ids);
    void kill_remote_hlc(unsigned int hlc_id);

    std::vector<unsigned int> get_active_vehicle_ids();
    std::vector<unsigned int> get_vehicle_ids_realtime();
    std::vector<unsigned int> get_vehicle_ids_simulated();

    //UI functions
    void set_sensitive(bool is_sensitive);

    //Set command line parameters
    bool cmd_simulated_time;
    unsigned int cmd_domain_id;
    std::string cmd_dds_initial_peer;

    //File chooser to select script(s) + location
    void open_file_explorer();
    void file_explorer_callback(std::string file_string, bool has_file);
    std::shared_ptr<FileChooserUI> file_chooser_window;

    //Vehicle button toggles
    void select_all_vehicles_real();
    void select_all_vehicles_sim();
    void select_no_vehicles();

    //Helper functions
    void get_path_name(std::string& in, std::string& out_path, std::string& out_name);
    bool session_exists(std::string session_id);
    void kill_session(std::string session_id);

    //Function to execute a shell command and get its output
    std::string execute_command(const char* cmd);

public:
    /**
     * \brief Constructor
     * \param _timer_ui Allows to access the timer UI, to reset the timer on system restart
     * \param _vehicleAutomatedControl Allows to send automated commands to the vehicles, like stopping them at their current position after simulation
     * \param argc Command line argument (from main())
     * \param argv Command line argument (from main())
     */
    SetupViewUI(std::shared_ptr<TimerViewUI> _timer_ui, std::shared_ptr<VehicleAutomatedControl> _vehicle_control, std::function<std::vector<uint8_t>()> _get_hlc_ids, unsigned int argc, char *argv[]);
    ~SetupViewUI();

    Gtk::Widget* get_parent();
};