#pragma once

#include "defaults.hpp"
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/monitoring/MonitoringUi.hpp"
#include "ui/map_view/MapViewUi.hpp"



class MainWindow
{
    Glib::RefPtr<Gtk::Builder> builder_master_layout;

    Gtk::Window* window_LCC = nullptr;
    Gtk::MenuBar* menu_bar = nullptr;
    Gtk::Box* box = nullptr;
    Gtk::Paned* pane1 = nullptr;
    Gtk::Paned* pane2 = nullptr;

public:
    Gtk::Window& get_window(); 

    MainWindow(
        std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi,
        std::shared_ptr<MonitoringUi> monitoringUi,
        std::shared_ptr<MapViewUi> mapViewUi
    );
};