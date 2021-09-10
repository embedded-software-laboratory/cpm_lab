#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "cpm/CommandLineReader.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/Logging.hpp"
#include "../../src/commonroad_classes/CommonRoadScenario.hpp"
#include "ObstacleSimulationManager.hpp"
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
#include "LaneletModelRecord.hpp"

/**
 * \brief This UI class is responsible for the Commonroad Tab in the LCC
 * It is used to load a commonroad file (and resize / transform it, if necessary)
 * \ingroup lcc_ui
 */
class CommonroadViewUI
{
private:
    //! Shared pointer to modify the current commonroad scenario, e.g. for (re-)loading or transformations
    std::shared_ptr<CommonRoadScenario> commonroad_scenario;

    //! Shared pointer to start a preview of the scenario movement
    std::shared_ptr<ObstacleSimulationManager> obstacle_sim_manager;

    //! Builder and pointer to UI elements
    Glib::RefPtr<Gtk::Builder> builder;

    //! Parent UI element of the window (or tab), scrollable
    Gtk::ScrolledWindow* parent = nullptr;
    //! Contains all UI elements in form of a list
    Gtk::Widget* commonroad_box = nullptr;

    //Commonroad path and parameters
    //! Entry for commonroad scenario file path
    Gtk::Entry* commonroad_path = nullptr;
    //! Entry to set or see the time step size
    Gtk::Entry* entry_time_step_size = nullptr;
    //! Entry to set the lane width for the smallest lane, which in turn scales the whole scenario
    Gtk::Entry* entry_lane_width = nullptr;
    //! Entry to translate the scenario in x direction
    Gtk::Entry* entry_translate_x = nullptr;
    //! Entry to translate the scenario in y direction
    Gtk::Entry* entry_translate_y = nullptr;
    //! Entry to rotate the scenario
    Gtk::Entry* entry_rotate = nullptr;
    
    //! Button to choose a commonroad file/scenario
    Gtk::Button* button_choose_commonroad = nullptr;
    //! Button to load the chosen file
    Gtk::Button* button_load_commonroad = nullptr;
    //! Button to load the transformation profile for the currently loaded scenario, if it exists
    Gtk::Button* button_load_profile = nullptr;
    //! Button to save the transformation profile for the currently loaded scenario
    Gtk::Button* button_save_profile = nullptr;
    //! Button to reset the transformation profile for the currently loaded scenario, if it exists
    Gtk::Button* button_reset_profile = nullptr;

    //! Button to apply a transformation set in the UI to the currently loaded scenario
    Gtk::Button* button_apply_transformation = nullptr;

    //! Button to preview the movement of the obstacles in the current scenario
    Gtk::Button* button_preview = nullptr;

    //Buttons to toggle views for the scenario
    //! Check button to (not) see traffic signs defined in the current scenario
    Gtk::CheckButton* check_traffic_signs;
    //! Check button to (not) see traffic lights defined in the current scenario
    Gtk::CheckButton* check_traffic_lights;
    //! Check button to (not) see descriptions of the lanelet type within each lanelet defined in the current scenario
    Gtk::CheckButton* check_lanelet_id;
    //! Check button to (not) see the orientation of each lanelet defined in the current scenario
    Gtk::CheckButton* check_lanelet_orientation;
    //! Check button to (not) see descriptions within each initial state defined in the current scenario
    Gtk::CheckButton* check_initial_state;
    //! Check button to (not) see descriptions within each goal state defined in the current scenario
    Gtk::CheckButton* check_goal_description;
    //! Check button to (not) see descriptions within each obstacle defined in the current scenario
    Gtk::CheckButton* check_obstacle_description;

    //View to set / edit which obstacles should be simulated / shown
    //! Box that includes toggles for static obstacles, to show them as obstacles or simulate their trajectory (aka point, as they are static)
    Gtk::FlowBox* static_obstacles_flowbox = nullptr;
    //! Box that includes toggles for dynamic obstacles, to show them as obstacles or simulate their trajectory
    Gtk::FlowBox* dynamic_obstacles_flowbox = nullptr; 
    //! List of dynamically created obstacle toggles for each static obstacle in the currently loaded scenario
    std::vector<std::shared_ptr<ObstacleToggle>> static_vehicle_toggles;
    //! List of dynamically created obstacle toggles for each dynamic obstacle in the currently loaded scenario
    std::vector<std::shared_ptr<ObstacleToggle>> dynamic_vehicle_toggles;
    //! When a new scenario was selected, redraw the obstacle lists in the dispatcher
    std::atomic_bool load_obstacle_list;
    /**
     * \brief Callback function for state changes in the toggles, which decide to show obstacles or simulate their trajectory.
     * Uses obstacle_sim_manager.
     * \param id Obstacle ID
     * \param state ToggleState of the toggle, which decides if the obstacle should be shown or if a trajectory should be simulated from the obstacle data
     */
    void vehicle_selection_changed(unsigned int id, ObstacleToggle::ToggleState state);

    /**
     * \brief This function gets called after the simulation manager was reset due to a transformation
     * It prevents the user from having to re-select which participants are simulated and which are supposed to be real
     */
    void apply_current_vehicle_selection();

    //! Treeview that shows information about planning problems
    Gtk::TreeView* problem_treeview;
    //! Scroll window for problem_treeview to scroll through planning problems
    Gtk::ScrolledWindow* problem_scrolled_window;
    //! TreeView "layout", data defintion to define rows
    ProblemModelRecord problem_record;
    //! Contains all current entries of problem_treeview
    Glib::RefPtr<Gtk::ListStore> problem_list_store;

    //! Treeview that shows information about planning problems
    Gtk::TreeView* lanelet_treeview;
    //! TreeView "layout", data defintion to define rows
    LaneletModelRecord lanelet_record;
    //! Contains all current entries of problem_treeview
    Glib::RefPtr<Gtk::ListStore> lanelet_list_store;
    
    /**
     * \brief Function to update the UI, running in ui_thread, calling the UI update dispatcher frequently
     */
    void update_ui();
    /**
     * \brief Callback for the UI update dispatcher, which connects to GTK's UI thread. All "timed" GTK UI changes must be performed here, to stay thread-safe.
     */
    void dispatcher_callback();
    //! From GTK, to communicate between thread and GUI
    Glib::Dispatcher ui_dispatcher;
    //! Thread to update the UI frequently
    std::thread ui_thread;
    //! Bool to stop the UI-update-thread as soon as the LCC gets closed
    std::atomic_bool run_thread;

    /**
     * \brief Callback for tooltip of problem_treeview (to show full message without scrolling)
     * \param x x coordinate of the mouse
     * \param y y coordinate of the mouse
     * \param keyboard_tooltip If activated by the keyboard
     * \param tooltip Reference to the GTK tooltip that will be shown
     */
    bool problem_tooltip_callback(int x, int y, bool keyboard_tooltip, const Glib::RefPtr<Gtk::Tooltip>& tooltip);
    /**
     * \brief Callback for tooltip of problem_treeview (to show full message without scrolling)
     * \param x x coordinate of the mouse
     * \param y y coordinate of the mouse
     * \param keyboard_tooltip If activated by the keyboard
     * \param tooltip Reference to the GTK tooltip that will be shown
     */
    bool lanelet_tooltip_callback(int x, int y, bool keyboard_tooltip, const Glib::RefPtr<Gtk::Tooltip>& tooltip);
    //! Variable for the reset action, to reload / reset all tables (for problem, lanelet etc. info) (is performed within the UI thread)
    std::atomic_bool reload_tables;

    //! Function to get the main window of the LCC, required e.g. to show dialogs properly
    std::function<Gtk::Window&()> get_main_window;

    /**
     * \brief Opens a file explorer to select a commonroad scenario
     */
    void open_file_explorer();
    /**
     * \brief File explorer callback, that tells if and which file was selected
     * \param file_string File path
     * \param has_file If a file was selected
     */
    void file_explorer_callback(std::string file_string, bool has_file);
    //! File chooser window to select script(s) + location, accessed by open_file_explorer and using file_explorer_callback
    std::shared_ptr<FileChooserUI> file_chooser_window;

    /**
     * \brief Function to load the chosen commonroad file
     * Displays an error message if this fails
     */
    void load_chosen_file();

    /**
     * \brief Callback for load button in UI
     */
    void load_button_callback();

    /**
     * \brief Transform text to double and return that value, if possible
     * \param value Text to transform
     * \param default_value Default value if transformation fails
     */
    double string_to_double(std::string value, double default_value);

    /**
     * \brief Function to apply the set transformation to the loaded scenario with a button click.
     * Without time scale, from all currently used text fields.
     */
    void apply_transformation();

    //Functions to apply one of the corresponding transformations after pressing enter within the entry
    /**
     * \brief Function to apply time scale when pressing enter in the according entry
     */
    bool apply_entry_time(GdkEventKey* event);
    /**
     * \brief Function to scale the scenario with the set value in the entry when pressing enter
     */
    bool apply_entry_scale(GdkEventKey* event);
    /**
     * \brief Function to translate the scenario in x direction with the set value in the entry when pressing enter
     */
    bool apply_entry_translate_x(GdkEventKey* event);
    /**
     * \brief Function to translate the scenario in y direction with the set value in the entry when pressing enter
     */
    bool apply_entry_translate_y(GdkEventKey* event);
    /**
     * \brief Function to rotate the scenario with the set value in the entry when pressing enter
     */
    bool apply_entry_rotate(GdkEventKey* event);

    //! A config file stores the previously selected commonroad scenario, so that one is always shown when the LCC is restarted. The key for accessing this data is stored in this variable.
    const std::string config_file_location = "commonroad";

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

    /**
     * \brief Callback for preview button, enables / disables preview of movement of commonroad obstacles
     */
    void preview_clicked();
    //! To start and stop a preview, we need to know if one is currently shown
    bool preview_enabled = false;

    //! Remember last click of load button, to prevent the user from "spamming" loading a file
    uint64_t last_scenario_load_timestamp{0};

public:
    /**
     * \brief Constructor
     * \param _commonroad_scenario The commonroad scenario to be managed by this view
     * \param obstacle_sim_manager To change the state of an obstacle in the obstacle manager, given its ID
     */
    CommonroadViewUI(
        std::shared_ptr<CommonRoadScenario> _commonroad_scenario,
        std::shared_ptr<ObstacleSimulationManager> obstacle_sim_manager
    );

    ~CommonroadViewUI() {
        run_thread.store(false);
        if (ui_thread.joinable())
            ui_thread.join();
    }

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

    /**
     * \brief In case the preview is stopped by a simulation start, reset the preview button label
     */
    void reset_preview();

    /**
     * \brief Get the parent widget to put the view in a parent container
     */
    Gtk::Widget* get_parent();
};