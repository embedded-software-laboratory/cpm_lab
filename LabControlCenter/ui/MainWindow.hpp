#pragma once

#include "defaults.hpp"
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"



class MainWindow
{
    Glib::RefPtr<Gtk::Builder> builder_master_layout;

    Gtk::Window* window_LCC = nullptr;
    Gtk::Paned* pane1 = nullptr;
    Gtk::Paned* pane2 = nullptr;

public:
    Gtk::Window& get_window(); 
    MainWindow(std::shared_ptr<VehicleManualControlUi> _vehicleManualControlUi);
};