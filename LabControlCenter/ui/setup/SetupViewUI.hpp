#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "VehicleManualControl.hpp"

//For popen
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

class SetupViewUI
{
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
    Gtk::Switch* switch_simulated_time = nullptr;
    Gtk::Switch* switch_launch_simulated_vehicles = nullptr;
    Gtk::Switch* switch_launch_cloud_discovery = nullptr;
    Gtk::Switch* switch_launch_ips = nullptr;
    Gtk::Switch* switch_launch_middleware = nullptr;

    Gtk::Button* button_deploy = nullptr;
    Gtk::Button* button_kill = nullptr;

    //Overall deploy functions
    void deploy_applications();
    void kill_deployed_applications();

    //Specific deploy functions
    void deploy_hlc_scripts();
    void deploy_middleware();
    void deploy_vehicles();
    void deploy_ips();
    void deploy_cloud_discovery();

    void kill_hlc_scripts();
    void kill_middleware();
    void kill_vehicles();
    void kill_ips();
    void kill_cloud_discovery();

    //UI functions
    void set_sensitive(bool is_sensitive);

    //Function to execute a shell command
    std::string execute_command(const char* cmd);

public:
    SetupViewUI();
    ~SetupViewUI();

    Gtk::Widget* get_parent();
};