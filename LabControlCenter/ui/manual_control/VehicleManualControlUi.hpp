#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "VehicleManualControl.hpp"



class VehicleManualControlUi
{
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::Window* window = nullptr;
    Gtk::Button* button_restart  = nullptr;
    Gtk::Button* button_stop = nullptr;
    Gtk::Entry* entry_js_device = nullptr;
    Gtk::Entry* entry_vehicle_id = nullptr;
    
    shared_ptr<VehicleManualControl> vehicleManualControl = nullptr;

public:
    VehicleManualControlUi(shared_ptr<VehicleManualControl> vehicleManualControl);

    Gtk::Window& get_window();
    
};