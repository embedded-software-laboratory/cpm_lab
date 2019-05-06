#pragma once

#include "defaults.hpp"
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/right_tabs/TabsViewUI.hpp"
#include "ui/monitoring/MonitoringUi.hpp"
#include "ui/map_view/MapViewUi.hpp"



class MainWindow
{
private:
    Glib::RefPtr<Gtk::Builder> builder_master_layout;

    Gtk::Window* window_LCC = nullptr;
    Gtk::Box* box = nullptr;
    Gtk::Paned* pane1 = nullptr;
    Gtk::Paned* pane2 = nullptr;

    //Menu Bar Items, signal handlers call ParamViewUI functions (using tabsViewUI)
    Gtk::MenuBar* menu_bar = nullptr;
    Gtk::MenuItem* menu_bar_params_reload = nullptr;
    Gtk::MenuItem* menu_bar_params_save = nullptr;
    Gtk::MenuItem* menu_bar_params_save_as = nullptr;
    Gtk::MenuItem* menu_bar_params_load_file = nullptr;
    Gtk::MenuItem* menu_bar_params_load_multiple_files = nullptr;
    Gtk::MenuItem* menu_bar_params_load_params = nullptr;

    //Signal handlers for menu bar items
    void on_menu_params_reload_pressed();
    void on_menu_params_save_pressed();
    void on_menu_params_save_as_pressed();
    void on_menu_params_load_file_pressed();
    void on_menu_params_load_multiple_files_pressed();
    void on_menu_params_load_params_pressed();

    std::shared_ptr<TabsViewUI> tabs_view_ui;
    std::shared_ptr<MonitoringUi> monitoring_ui;
    std::shared_ptr<MapViewUi> map_view_ui;

public:
    Gtk::Window& get_window(); 

    MainWindow(
        std::shared_ptr<TabsViewUI> tabsViewUI,
        std::shared_ptr<MonitoringUi> monitoringUi,
        std::shared_ptr<MapViewUi> mapViewUi
    );
};