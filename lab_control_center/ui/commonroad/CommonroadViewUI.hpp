#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "cpm/CommandLineReader.hpp"
#include "cpm/Logging.hpp"
#include "../../src/commonroad_classes/CommonRoadScenario.hpp"
#include "ui/file_chooser/FileChooserUI.hpp"

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

/**
 * \brief This UI class is responsible for the Commonroad Tab in the LCC
 * It is used to load a commonroad file (and resize / transform it, if necessary)
 */
class CommonroadViewUI
{
private:
    //Builder and pointer to UI elements
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::ScrolledWindow* parent = nullptr;
    Gtk::Widget* commonroad_box = nullptr;

    //Commonroad path and parameters
    Gtk::Entry* commonroad_path = nullptr;
    Gtk::Entry* entry_lane_width = nullptr;
    Gtk::Entry* entry_translate_x = nullptr;
    Gtk::Entry* entry_translate_y = nullptr;
    
    //Button to choose commonroad file
    Gtk::Button* button_choose_commonroad = nullptr;
    Gtk::Button* button_load_commonroad = nullptr;

    //Button to apply a transformation set in the UI to the currently loaded scenario permanently
    Gtk::Button* button_apply_transformation = nullptr;

    //Function to get the main window
    std::function<Gtk::Window&()> get_main_window;

    //Shared pointer to modify the current commonroad scenario
    std::shared_ptr<CommonRoadScenario> commonroad_scenario;

    //File chooser to select script(s) + location
    void open_file_explorer();
    void file_explorer_callback(std::string file_string, bool has_file);
    std::shared_ptr<FileChooserUI> file_chooser_window;

    /**
     * \brief Function to load the chosen commonroad file
     * Displays an error message if this fails
     */
    void load_chosen_file();

    //Transform text to double, if possible
    double string_to_double(std::string value, double default_value);

    //Function to apply the set transformation to the loaded scenario permanently
    void apply_transformation();

    //Functions to apply one of the corresponding transformations after pressing enter within the entry
    bool apply_entry_scale(GdkEventKey* event);
    bool apply_entry_translate_x(GdkEventKey* event);
    bool apply_entry_translate_y(GdkEventKey* event);

public:
    /**
     * \brief Constructor
     * \param _commonroad_scenario The commonroad scenario to be managed by this view
     */
    CommonroadViewUI(
        std::shared_ptr<CommonRoadScenario> _commonroad_scenario
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