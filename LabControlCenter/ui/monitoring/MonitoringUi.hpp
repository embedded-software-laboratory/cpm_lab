#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "defaults.hpp"

class MonitoringUi
{
    Glib::RefPtr<Gtk::Builder> builder;
    Gtk::Window* window;
    Gtk::Grid* grid_vehicle_monitor;
    
public:
    MonitoringUi();
    Gtk::Window& get_window();    
};