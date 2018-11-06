#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>


class MonitoringUi
{
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::Window* window = nullptr;
    
public:
    MonitoringUi();


    Gtk::Window& get_window();
    
};