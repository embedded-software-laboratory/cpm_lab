#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "VehicleManualControl.hpp"
#include "cpm/CommandLineReader.hpp"
#include "ui/file_chooser/FileChooserUI.hpp"
#include "ui/timer/TimerViewUI.hpp"

//For popen
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <array>

class SetupViewUI
{
private:
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::Widget* parent = nullptr;
    Gtk::Entry* script_path = nullptr;
    Gtk::Entry* script_name = nullptr;
    Gtk::ToggleButton* toggle_vehicle_1 = nullptr;
    Gtk::ToggleButton* toggle_vehicle_2 = nullptr;
    Gtk::ToggleButton* toggle_vehicle_3 = nullptr;
    Gtk::ToggleButton* toggle_vehicle_4 = nullptr;
    Gtk::ToggleButton* toggle_vehicle_5 = nullptr;
    Gtk::ToggleButton* toggle_vehicle_6 = nullptr;
    Gtk::Button* button_select_all_vehicles = nullptr;
    Gtk::Button* button_select_no_vehicles = nullptr;
    Gtk::Button* button_choose_script = nullptr;
    Gtk::Switch* switch_simulated_time = nullptr;
    Gtk::Switch* switch_launch_simulated_vehicles = nullptr;
    Gtk::Switch* switch_launch_cloud_discovery = nullptr;
    Gtk::Switch* switch_launch_ips = nullptr;
    Gtk::Switch* switch_launch_middleware = nullptr;

    Gtk::Button* button_deploy = nullptr;
    Gtk::Button* button_kill = nullptr;

    //Timer function - replace current timer in the whole system when user switches between simulated and real time
    std::shared_ptr<TimerViewUI> timer_ui;
    void switch_timer_set();

    //Overall deploy functions
    void deploy_applications();
    void kill_deployed_applications();

    //Specific deploy functions
    void deploy_hlc_scripts();
    void deploy_middleware();
    void deploy_vehicles();
    void deploy_vehicle(int id);
    void deploy_ips();
    void deploy_cloud_discovery();

    void kill_hlc_scripts();
    void kill_middleware();
    void kill_vehicles();
    void kill_vehicle(int id);
    void kill_ips();
    void kill_cloud_discovery();

    std::vector<int> get_active_vehicle_ids();

    //UI functions
    void set_sensitive(bool is_sensitive);

    //Function to execute a shell command
    std::string execute_command(const char* cmd);

    //Set command line parameters
    bool cmd_simulated_time;
    int cmd_domain_id;
    std::string cmd_dds_initial_peer;

    //File chooser to select script(s) + location
    void open_file_explorer();
    void file_explorer_callback(std::string file_string, bool has_file);
    std::shared_ptr<FileChooserUI> file_chooser_window;

    //Vehicle button toggles
    void select_all_vehicles();
    void select_no_vehicles();

public:
    SetupViewUI(std::shared_ptr<TimerViewUI> _timer_ui, int argc, char *argv[]);
    ~SetupViewUI();

    Gtk::Widget* get_parent();
};