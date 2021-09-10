#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <gtkmm/grid.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <math.h>

#include "TimeSeries.hpp"
#include "defaults.hpp"
#include "cpm/Logging.hpp"
#include "cpm/get_time_ns.hpp"
#include "ui/setup/Deploy.hpp"

#include "TrajectoryInterpolation.hpp"

#include "ui/setup/CrashChecker.hpp"

using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;
using VehicleTrajectories = map<uint8_t, VehicleCommandTrajectory >;

/**
 * \class MonitoringUi
 * \brief UI class for the monitoring part on the bottom of the LCC's UI, below the map view. Shows detailed vehicle information (i.e. battery etc.), RTT info, 
 * currently connected NUCs, reboot options for the NUCs
 * \ingroup lcc_ui
 */
class MonitoringUi
{
public:
    //! GTK UI builder
    Glib::RefPtr<Gtk::Builder> builder;
    //! Parent element, which contains all other elements and can itself be contained in another element
    Gtk::Box* parent;
    //! Viewport that contains grid_vehicle_monitor
    Gtk::Viewport* viewport_monitoring;
    //! Contains the actual vehicle information (like table entries), is placed in viewport_monitoring
    Gtk::Grid* grid_vehicle_monitor;
    //! Contains all information that are shown on top of grid_vehicle_monitor / the vehicle information, e.g. RTT, reset button etc.
    Gtk::Box* box_buttons;
    //! Button to reset the vehicle information shown in grid_vehicle_monitor
    Gtk::Button* button_reset_view;
    //! Short label to show how many HLCs are online
    Gtk::Label* label_hlc_description_short;
    //! Label that lists all IDs of HLCs that are online
    Gtk::Label* label_hlc_description_long;
    //! Entry to give one or more HLC IDs of HLCs that should be rebooted
    Gtk::Entry* entry_hlc_reboot;
    //! Button to send a reboot command to all HLCs specified in entry_hlc_reboot
    Gtk::Button* button_hlc_reboot;
    //! Shows short version of current RTT for the HLCs
    Gtk::Label* label_rtt_hlc_short;
    //! Shows detailed RTT information for the HLCs
    Gtk::Label* label_rtt_hlc_long;
    //! Shows short version of current RTT for the vehicles
    Gtk::Label* label_rtt_vehicle_short;
    //! Shows detailed RTT information for the vehicles
    Gtk::Label* label_rtt_vehicle_long;
    //! Shows the current runtime of the simulation (Time since deploy)
    Gtk::Label* label_experiment_time;
    //! Provides a reference to deploy functions, for rebooting the vehicles
    std::shared_ptr<Deploy> deploy_functions;
    //! To check if a NUC crashed
    std::weak_ptr<CrashChecker> crash_checker;
    //! To show data of all currently active vehicles in grid_vehicle_monitoring
    std::function<VehicleData()> get_vehicle_data;
    //! To get currently online HLC IDs
    std::function<std::vector<uint8_t>()> get_hlc_data;
    //! To show, in the vehicle overview, if the vehicle is mapped to a HLC during simulation
    std::function<std::pair<bool, std::map<uint32_t, uint8_t>>()> get_vehicle_to_hlc_mapping;
    //! Reset time series data to get rid of potential outdated vehicle information
    std::function<void()> reset_data;
    //! Get vehicle trajectories to obtain the reference deviation
    std::function<VehicleTrajectories()> get_vehicle_trajectory;
    //! Get current RTT measurements to display HLC and vehicle RTT information
    std::function<bool(std::string, uint64_t&, uint64_t&, uint64_t&, double&)> get_rtt_values;
    //! To stop the experiment in case of errors, e.g. if a NUC disconnected
    std::function<void()> kill_deployed_applications; 

    /**
     * \brief Calls update_dispatcher every 200ms to update the GUI
     */
    void ui_update_loop();
    //! To communicate between thread and GUI
    Glib::Dispatcher update_dispatcher;
    //! Thread that calls ui_update_loop
    std::thread ui_thread;
    //! Stop condition for ui_thread (stop if false)
    std::atomic_bool run_thread;

    //! To measure how long the simulation has been running
    std::atomic_uint64_t sim_start_time;

    //! Full rows for the vehicles, usually not all information are shown
    const vector<string> rows = { "battery_voltage", "battery_level", "last_msg_state", "clock_delta", "pose_x", "pose_y", "pose_yaw", "ips_x", "ips_y", "ips_yaw", "odometer_distance", "imu_acceleration_forward", "imu_acceleration_left", "speed", "motor_current" };
    //! We do not want to show all vehicle information to the user - empty string become empty rows (better formatting)
    const vector<string> rows_restricted = {"battery_level", "ips_dt", "last_msg_state", "clock_delta", "reference_deviation", "speed", "nuc_connected"};

    /**
     * \brief List of currently shown vehicle IDs in grid_vehicle_monitor (which does not have a "get_all"-function) to delete no longer existing vehicles.
     * As "query_child" did not exist due to my compiler (even though it does in the specs) and as there is no efficient way to iterate through
     * all GTK::Grid childs, the only option left was to mimic the behaviour of the grid to determine the current position of an entry.
     * The positions (columns) change as soon as one entry gets deleted, so using the vehicle id to get the position no longer works.
     * Instead, the IDs are stored in or removed from a vector, and the position of the IDs in the vector should always be the same as the position
     * or row number in the grid. Helper functions are used to compensate further.
     */
    std::vector<uint8_t> grid_vehicle_ids;

    /**
     * \brief Uses grid_vehicle_ids to determine the position of the vehicle_id-entry in the GTK Grid.
     * DO NOT USE if the entry does not exist.
     * \param vehicle_id The vehicle ID to get the GTK column ID for
     */
    int get_column_id(uint8_t vehicle_id);

    //! Indicates the starting time of the last error occurance for each vehicle
    vector<vector<uint64_t> > error_timestamps{vector<vector<uint64_t> > (rows_restricted.size(), vector<uint64_t>(30,0))};

    //! Indicates if an error already triggered a kill 
    vector<vector<bool> > error_triggered{vector<vector<bool> > (rows_restricted.size(), vector<bool>(30,false))};
    
    /**
     * \brief Init is also called when the object is constructed. It initializes the ui thread, callbacks etc to update the ui regularly when a new vehicle connects
     */
    void init_ui_thread();

    /**
     * \brief This resets the ui thread, to get rid of old vehicle boxes (if the user desires to get rid of them)
     */
    void reset_ui_thread();

    /**
     * \brief Called when resetting the ui or when the object gets deleted - kills the currently running thread
     */
    void stop_ui_thread();

public:
    /**
     * \brief Constructor, creates the monitoring view for detailed vehicle information, rebooting HLCs, RTT info etc...
     * \param deploy_functions_callback Provides a reference to deploy functions, for rebooting the vehicles
     * \param get_vehicle_data_callback To show data of all currently active vehicles in grid_vehicle_monitoring
     * \param get_hlc_data_callback To get currently online HLC IDs
     * \param get_vehicle_trajectory_command_callback Get vehicle trajectories to obtain the reference deviation
     * \param reset_data_callback Reset time series data to get rid of potential outdated vehicle information
     * \param get_rtt_values Get current RTT measurements to display HLC and vehicle RTT information
     * \param kill_deployed_applications_callback To stop the experiment in case of errors, e.g. if a NUC disconnected
     */
    explicit MonitoringUi(
        std::shared_ptr<Deploy> deploy_functions_callback,  
        std::function<VehicleData()> get_vehicle_data_callback, 
        std::function<std::vector<uint8_t>()> get_hlc_data_callback,
        std::function<VehicleTrajectories()> get_vehicle_trajectory_command_callback, 
        std::function<void()> reset_data_callback,
        std::function<bool(std::string, uint64_t&, uint64_t&, uint64_t&, double&)> get_rtt_values,
        std::function<void()> kill_deployed_applications_callback 
    );

    /**
     * \brief Destructor, stops the UI update thread
     */
    ~MonitoringUi();

    /**
     * \brief Function to get the parent widget, so that this UI element can be placed within another UI element
     */
    Gtk::Box* get_parent();

    /**
     * \brief Clears currently shown information in the vehicle view / detailed vehicle information table
     */
    void reset_vehicle_view();

    /**
     * \brief Needs to be obtained after set up, as this function cannot be provided before. Sets up the function that returns the current vehicle to HLC mapping
     * \param get_vehicle_to_hlc_mapping To show, in the vehicle overview, if the vehicle is mapped to a HLC during simulation
     */
    void register_vehicle_to_hlc_mapping(std::function<std::pair<bool, std::map<uint32_t, uint8_t>>()> get_vehicle_to_hlc_mapping);

    /**
     * \brief Checker needs to be set up in SetupView, and SetupView requires access to monitoring, so we have to do this after construction.
     * A weak_ptr must be used in order to avoid cyclic dependencies (the destructor was not called when using a shared_ptr).
     * \param _crash_checker Reference to a crash checker object, required to see if a HLC crashed
     */
    void register_crash_checker(std::weak_ptr<CrashChecker> _crash_checker);

    /**
     * \brief Function to call when the simulation starts, to reset data structures, start timers etc
     */
    void notify_sim_start();

    /**
     * \brief Function to call when the simulation stops, to reset data structures, start timers etc
     */
    void notify_sim_stop();
};