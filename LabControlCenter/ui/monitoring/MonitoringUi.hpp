#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

#include <atomic>
#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "TimeSeries.hpp"
#include "defaults.hpp"
#include "cpm/Logging.hpp"
#include "ui/setup/Deploy.hpp"


using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;
using VehicleTrajectories = map<uint8_t, map<uint64_t, TrajectoryPoint> >;

class MonitoringUi
{
public:
    Glib::RefPtr<Gtk::Builder> builder;
    Gtk::Box* parent;
    Gtk::Viewport* viewport_monitoring;
    Gtk::Grid* grid_vehicle_monitor;
    Gtk::Box* box_buttons;
    Gtk::Button* button_reset_view;
    Gtk::Label* label_hlc_description_short;
    Gtk::Label* label_hlc_description_long;
    std::shared_ptr<Deploy> deploy_functions;
    std::function<VehicleData()> get_vehicle_data;
    std::function<std::vector<std::string>()> get_hlc_data;
    std::function<void()> reset_data;
    std::function<VehicleTrajectories()> get_vehicle_trajectory;

    std::string reboot_script = "bash reboot_raspberry.bash 192.168.1.1";
    
    //Before: TimerFD, but this class is stopped by stop signals which might be emitted multiple times by the LCC depending on user interaction
    //Thus: Own timer implementation instead
    void ui_update_loop();
    Glib::Dispatcher update_dispatcher; //to communicate between thread and GUI
    std::thread ui_thread;
    std::atomic_bool run_thread;

    //full rows
    const vector<string> rows = { "battery_voltage", "battery_level", "clock_delta", "pose_x", "pose_y", "pose_yaw", "ips_x", "ips_y", "ips_yaw", "odometer_distance", "imu_acceleration_forward", "imu_acceleration_left", "speed", "motor_current" };
    //We do not want to show all vehicle information to the user - empty string become empty rows (better formatting)
    const vector<string> rows_restricted = {"ips", "battery_level", "clock_delta", "speed", "pose_x", "pose_y"};
    
    //Init is also called when the object is constructed. It initializes the ui thread, callbacks etc to update the ui regularly when a new vehicle connects
    void init_ui_thread();

    //This resets the ui thread, to get rid of old vehicle boxes (if the user desires to get rid of them)
    void reset_ui_thread();

    //Called when resetting the ui or when the object gets deleted - kills the currently running thread
    void stop_ui_thread();

public:
    explicit MonitoringUi(
        std::shared_ptr<Deploy> _deploy_functions,  
        std::function<VehicleData()> get_vehicle_data_callback, 
        std::function<std::vector<std::string>()> get_hlc_data_callback,
        std::function<VehicleTrajectories()> _get_vehicle_trajectory_command_callback, 
        std::function<void()> reset_data_callback
    );
    ~MonitoringUi();
    Gtk::Box* get_parent();
    void reset_vehicle_view();
};