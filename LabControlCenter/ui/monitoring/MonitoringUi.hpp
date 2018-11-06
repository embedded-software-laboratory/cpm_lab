#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "defaults.hpp"

class MonitoringUi
{
    Glib::RefPtr<Gtk::Builder> builder;
    Gtk::Window* window = nullptr;
    
public:
    MonitoringUi();
    Gtk::Window& get_window();    
};