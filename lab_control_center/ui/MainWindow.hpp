// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#pragma once

#include "defaults.hpp"
#include <memory>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/right_tabs/TabsViewUI.hpp"
#include "ui/monitoring/MonitoringUi.hpp"
#include "ui/map_view/MapViewUi.hpp"
#include "ui/params/ParamViewUI.hpp"
#include "ui/file_chooser/FileChooserUI.hpp"
#include "ui/file_chooser/FileSaverUI.hpp"


/**
 * \class MainWindow
 * \brief LCC UI "top" class that contains all other UI elements
 * \ingroup lcc_ui
 */
class MainWindow
{
private:
    //! GTK UI Builder
    Glib::RefPtr<Gtk::Builder> builder_master_layout;

    //! Main LCC window object
    Gtk::Window* window_LCC = nullptr;
    //! Box that divides the main window
    Gtk::Box* box = nullptr;
    //! The panes allow to separate the view into Map, Tabs on the right and Monitoring on the Bottom
    Gtk::Paned* pane1 = nullptr;
    //! The panes allow to separate the view into Map, Tabs on the right and Monitoring on the Bottom
    Gtk::Paned* pane2 = nullptr;
    //! Makes the monitoring view scrollabel
    Gtk::ScrolledWindow* monitoring_scroll_window = nullptr;

    //Menu Bar Items, signal handlers call ParamViewUI functions (using tabsViewUI)
    //! Menu Bar, which e.g. contains param. buttons
    Gtk::MenuBar* menu_bar = nullptr;
    //! Menu Bar Item that allows to reload currently loaded parameters in ParamViewUI
    Gtk::MenuItem* menu_bar_params_reload = nullptr;
    //! Menu Bar Item that allows to save currently loaded parameters in ParamViewUI
    Gtk::MenuItem* menu_bar_params_save = nullptr;
    //! Menu Bar Item that allows to save currently loaded parameters in ParamViewUI in a new file
    Gtk::MenuItem* menu_bar_params_save_as = nullptr;
    //! Menu Bar Item that allows to load new parameters in ParamViewUI
    Gtk::MenuItem* menu_bar_params_load_file = nullptr;
    // Gtk::MenuItem* menu_bar_params_load_multiple_files = nullptr;
    // Gtk::MenuItem* menu_bar_params_load_params = nullptr;
    //! Menu Bar Item to rotate the MapView to the left by 90 degrees
    Gtk::MenuItem* menu_bar_mapview_rotate_left = nullptr;
    //! Menu Bar Item to rotate the MapView to the right by 90 degrees
    Gtk::MenuItem* menu_bar_mapview_rotate_right = nullptr;

    //Signal handlers for menu bar items
    /**
     * \brief Signal handler for menu bar reload button, 
     * allows to reload currently loaded parameters in ParamViewUI
     */
    void on_menu_params_reload_pressed();
    /**
     * \brief Signal handler for menu bar save button, 
     * allows to save currently loaded parameters in ParamViewUI
     */
    void on_menu_params_save_pressed();
    /**
     * \brief Signal handler for menu bar save as button, 
     * allows to save currently loaded parameters in ParamViewUI in a new file
     */
    void on_menu_params_save_as_pressed();
    /**
     * \brief Signal handler for menu bar load button, 
     * allows to load new parameters in ParamViewUI
     */
    void on_menu_params_load_file_pressed();
    // void on_menu_params_load_multiple_files_pressed();
    // void on_menu_params_load_params_pressed();
    /**
     * \brief Signal handler for menu bar rotate left button, 
     * allows to rotate the MapView to the left by 90 degrees
     */
    void on_menu_mapview_rotate_left_pressed();
    /**
     * \brief Signal handler for menu bar rotate right button, 
     * allows to rotate the MapView to the right by 90 degrees
     */
    void on_menu_mapview_rotate_right_pressed();

    //! File chooser window, which is opened when parameters are loaded
    std::shared_ptr<FileChooserUI> file_chooser_window;
    /**
     * \brief Callback for file chooser for parameters
     * \param file_string File (path) to load
     * \param has_file True if the user selected a valid file, else ignore file_string
     */
    void file_chooser_callback(std::string file_string, bool has_file);
    //! File saver window, which is opened when parameters are saved
    std::shared_ptr<FileSaverUI> file_saver_window;
    /**
     * \brief Callback for file saver for parameters
     * \param file_string File (path) to save to
     * \param has_file True if the user selected a valid file, else ignore file_string
     */
    void file_saver_callback(std::string file_string, bool has_file);

    //! Provides access to tabs that contain UI content like parameters, commonroad, setup..., put in the right of the main window
    std::shared_ptr<TabsViewUI> tabs_view_ui;
    //! Monitoring view on the bottom, which e.g. contains vehicle information
    std::shared_ptr<MonitoringUi> monitoring_ui;
    //! Map view on the left / center, which shows the map and vehicles, obstacles, ...
    std::shared_ptr<MapViewUi> map_view_ui;
    //! Weak pointer to the object for the Param View/Edit/Create Tab. Shared ptr lead to destructors not being called.
    std::weak_ptr<ParamViewUI> param_view_ui;

public:
    /**
     * \brief Function that returns the Main Window of the LCC. 
     * Can e.g. be used to open / close that window (or as in main: app->run(...))
     */
    Gtk::Window& get_window(); 

    /**
     * \brief Constructor for the LCC's main window
     * \param tabsViewUI UI element for the tabs shown on the right, that e.g. contain the setup, parameter and timer view
     * \param monitoringUi UI element for the monitoring view on the bottom, that e.g. contains vehicle information
     * \param mapViewUi UI element for the map view, which e.g. shows the currently loaded map and vehicles
     * \param paramViewUI Shared pointer to the object for the Param View/Edit/Create Tab
     */
    MainWindow(
        std::shared_ptr<TabsViewUI> tabsViewUI,
        std::shared_ptr<MonitoringUi> monitoringUi,
        std::shared_ptr<MapViewUi> mapViewUi,
        std::shared_ptr<ParamViewUI> paramViewUI
    );
};