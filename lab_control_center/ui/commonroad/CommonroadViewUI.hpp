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

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "cpm/CommandLineReader.hpp"
#include "cpm/Logging.hpp"
#include "../../src/commonroad_classes/CommonRoadScenario.hpp"
#include "ui/file_chooser/FileChooserUI.hpp"
#include "ui/commonroad/ObstacleToggle.hpp"

#include <atomic>
#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

#include "ProblemModelRecord.hpp"

/**
 * \brief This UI class is responsible for the Commonroad Tab in the LCC
 * It is used to load a commonroad file (and resize / transform it, if necessary)
 */
class CommonroadViewUI
{
private:
    //Shared pointer to modify the current commonroad scenario
    std::shared_ptr<CommonRoadScenario> commonroad_scenario;

    //Builder and pointer to UI elements
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::ScrolledWindow* parent = nullptr;
    Gtk::Widget* commonroad_box = nullptr;

    //Commonroad path and parameters
    Gtk::Entry* commonroad_path = nullptr;
    Gtk::Entry* entry_time_step_size = nullptr;
    Gtk::Entry* entry_lane_width = nullptr;
    Gtk::Entry* entry_translate_x = nullptr;
    Gtk::Entry* entry_translate_y = nullptr;
    
    //Button to choose commonroad file
    Gtk::Button* button_choose_commonroad = nullptr;
    Gtk::Button* button_load_commonroad = nullptr;
    Gtk::Button* button_load_profile = nullptr;
    Gtk::Button* button_save_profile = nullptr;
    Gtk::Button* button_reset_profile = nullptr;

    //Button to apply a transformation set in the UI to the currently loaded scenario permanently
    Gtk::Button* button_apply_transformation = nullptr;

    //View to set / edit which obstacles should be simulated / shown
    Gtk::FlowBox* static_obstacles_flowbox = nullptr;
    Gtk::FlowBox* dynamic_obstacles_flowbox = nullptr; 
    std::vector<std::shared_ptr<ObstacleToggle>> static_vehicle_toggles;
    std::vector<std::shared_ptr<ObstacleToggle>> dynamic_vehicle_toggles;
    std::atomic_bool load_obstacle_list; //When a new scenario was selected, redraw the obstacle lists in the dispatcher
    //Callback function for state changes in the toggles
    void vehicle_selection_changed(unsigned int id, ObstacleToggle::ToggleState state);
    std::function<void(int, ObstacleToggle::ToggleState state)> set_obstacle_manager_obstacle_state;

    /**
     * \brief This function gets called after the simulation manager was reset due to a transformation
     * It prevents the user from having to re-select which participants are simulated and which are supposed to be real
     */
    void apply_current_vehicle_selection();

    //Treeview that shows information about planning problems
    Gtk::TreeView* problem_treeview;
    Gtk::ScrolledWindow* problem_scrolled_window;
    //TreeView Layout, status storage for the UI
    ProblemModelRecord problem_record;
    Glib::RefPtr<Gtk::ListStore> problem_list_store;
    //UI update functions and objects
    void update_ui();
    void dispatcher_callback();
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::thread ui_thread;
    std::atomic_bool run_thread;
    //Callback for tooltip (to show full message without scrolling)
    bool tooltip_callback(int x, int y, bool keyboard_tooltip, const Glib::RefPtr<Gtk::Tooltip>& tooltip);
    //Variable for the reset action (is performed within the UI)
    std::atomic_bool reload_problems;

    //Function to get the main window
    std::function<Gtk::Window&()> get_main_window;

    //File chooser to select script(s) + location
    void open_file_explorer();
    void file_explorer_callback(std::string file_string, bool has_file);
    std::shared_ptr<FileChooserUI> file_chooser_window;

    /**
     * \brief Function to load the chosen commonroad file
     * Displays an error message if this fails
     */
    void load_chosen_file();

    //Callback for load button in UI
    void load_button_callback();

    //Transform text to double, if possible
    double string_to_double(std::string value, double default_value);

    //Function to apply the set transformation to the loaded scenario with a button click
    void apply_transformation(); //Without time scale, from all currently used text fields

    //Functions to apply one of the corresponding transformations after pressing enter within the entry
    bool apply_entry_time(GdkEventKey* event);
    bool apply_entry_scale(GdkEventKey* event);
    bool apply_entry_translate_x(GdkEventKey* event);
    bool apply_entry_translate_y(GdkEventKey* event);

    //Config file that stores the previously selected script
    const std::string config_file_location = "./commonroad_file_chooser.config";

    /**
     * \brief Load, if exists, the stored transformation for the current file
     */
    void load_transformation_from_profile();

    /**
     * \brief Store the transform profile that stores previously used transformations for commonroad scenarios
     */
    void store_transform_profile();

    /**
     * \brief Reset transform profile for the currently selected files
     */
    void reset_current_transform_profile();

public:
    /**
     * \brief Constructor
     * \param _commonroad_scenario The commonroad scenario to be managed by this view
     * \param _set_obstacle_manager_obstacle_state Callback function to change the state of an obstacle in the obstacle manager, given its ID
     */
    CommonroadViewUI(
        std::shared_ptr<CommonRoadScenario> _commonroad_scenario,
        std::function<void(int, ObstacleToggle::ToggleState state)> _set_obstacle_manager_obstacle_state
    );

    /**
     * \brief Set the callback function that returns a reference to the application's main window
     * \param _get_main_window Returns reference to the main window, which is "needed" for window creation (works without it, but not without Gtk complaining in the terminal)
     */
    void set_main_window_callback(std::function<Gtk::Window&()> _get_main_window);


    /**
     * \brief Set responsiveness of this part of the UI - should be unresponsive whenever the simulation is running, thus this function is given as callback to the setup UI part
     * \param is_sensitive True if this sub-window should be responsive to user input, else false
     */
    void set_sensitive(bool is_sensitive);

    //Get the parent widget to put the view in a parent container
    Gtk::Widget* get_parent();
};