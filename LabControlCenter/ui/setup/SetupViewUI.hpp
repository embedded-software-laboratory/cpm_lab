#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "VehicleManualControl.hpp"



class SetupViewUI
{
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::Widget* parent = nullptr;
    Gtk::Entry* script_path = nullptr;
    Gtk::Entry* script_name = nullptr;
    Gtk::ToggleBox* toggle_vehicle_1 = nullptr;
    Gtk::ToggleBox* toggle_vehicle_2 = nullptr;
    Gtk::ToggleBox* toggle_vehicle_3 = nullptr;
    Gtk::ToggleBox* toggle_vehicle_4 = nullptr;
    Gtk::ToggleBox* toggle_vehicle_5 = nullptr;
    Gtk::ToggleBox* toggle_vehicle_6 = nullptr;
    Gtk::Button* button_select_all_vehicles = nullptr;
    Gtk::Button* button_select_no_vehicles = nullptr;
    Gtk::Switch* switch_simulated_time = nullptr;
    
    shared_ptr<VehicleManualControl> vehicleManualControl = nullptr;

public:
    SetupViewUI(shared_ptr<VehicleManualControl> vehicleManualControl);

    Gtk::Widget* get_parent();
};