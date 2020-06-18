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
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/right_tabs/TabsViewUI.hpp"
#include "ui/monitoring/MonitoringUi.hpp"
#include "ui/map_view/MapViewUi.hpp"
#include "ui/file_chooser/FileChooserUI.hpp"
#include "ui/file_chooser/FileSaverUI.hpp"


class MainWindow
{
private:
    Glib::RefPtr<Gtk::Builder> builder_master_layout;

    Gtk::Window* window_LCC = nullptr;
    Gtk::Box* box = nullptr;
    Gtk::Paned* pane1 = nullptr;
    Gtk::Paned* pane2 = nullptr;
    Gtk::ScrolledWindow* monitoring_scroll_window = nullptr;

    //Menu Bar Items, signal handlers call ParamViewUI functions (using tabsViewUI)
    Gtk::MenuBar* menu_bar = nullptr;
    Gtk::MenuItem* menu_bar_params_reload = nullptr;
    Gtk::MenuItem* menu_bar_params_save = nullptr;
    Gtk::MenuItem* menu_bar_params_save_as = nullptr;
    Gtk::MenuItem* menu_bar_params_load_file = nullptr;
    // Gtk::MenuItem* menu_bar_params_load_multiple_files = nullptr;
    // Gtk::MenuItem* menu_bar_params_load_params = nullptr;

    //Signal handlers for menu bar items
    void on_menu_params_reload_pressed();
    void on_menu_params_save_pressed();
    void on_menu_params_save_as_pressed();
    void on_menu_params_load_file_pressed();
    // void on_menu_params_load_multiple_files_pressed();
    // void on_menu_params_load_params_pressed();

    //File chooser window
    std::shared_ptr<FileChooserUI> file_chooser_window;
    void file_chooser_callback(std::string file_string, bool has_file);
    std::shared_ptr<FileSaverUI> file_saver_window;
    void file_saver_callback(std::string file_string, bool has_file);

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