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

    std::string reboot_script = "bash ./bash/reboot_raspberry.bash 192.168.1.1";

    std::vector<std::thread> reboot_threads; //threads that are responsible for rebooting vehicles    
    std::atomic_uint8_t thread_count; //thread counter, set before thread creation so that, if they finish before the next one is created, still threads are only joined after all threads that need to be created have finished their work
    void notify_reboot_finished(); //notify function that gets called by the threads when they have finished their work
    std::mutex notify_callback_in_use; //the notify_reboot_finished function should only be accessible by one thread at once, thus use this mutex
    size_t notify_count; //counter for notify_finished; if it does not match thread_count after all threads have called it, print an error message (means that there was a setup mistake made at thread creation)
    void kill_all_threads(); //function to join all threads
    
    //Before: TimerFD, but this class is stopped by stop signals which might be emitted multiple times by the LCC depending on user interaction
    //Thus: Own timer implementation instead
    void ui_update_loop();
    Glib::Dispatcher update_dispatcher; //to communicate between thread and GUI
    std::thread ui_thread;
    std::atomic_bool run_thread;

    //full rows
    const vector<string> rows = { "battery_voltage", "battery_level", "clock_delta", "pose_x", "pose_y", "pose_yaw", "ips_x", "ips_y", "ips_yaw", "odometer_distance", "imu_acceleration_forward", "imu_acceleration_left", "speed", "motor_current" };
    //We do not want to show all vehicle information to the user - empty string become empty rows (better formatting)
    const vector<string> rows_restricted = {"ips", "battery_level", "clock_delta", "reference_deviation", "speed"};
    
    //Init is also called when the object is constructed. It initializes the ui thread, callbacks etc to update the ui regularly when a new vehicle connects
    void init_ui_thread();

    //This resets the ui thread, to get rid of old vehicle boxes (if the user desires to get rid of them)
    void reset_ui_thread();

    //Called when resetting the ui or when the object gets deleted - kills the currently running thread
    void stop_ui_thread();

public:
    explicit MonitoringUi(
        std::shared_ptr<Deploy> deploy_functions_callback,  
        std::function<VehicleData()> get_vehicle_data_callback, 
        std::function<std::vector<std::string>()> get_hlc_data_callback,
        std::function<VehicleTrajectories()> get_vehicle_trajectory_command_callback, 
        std::function<void()> reset_data_callback
    );
    ~MonitoringUi();
    Gtk::Box* get_parent();
    void reset_vehicle_view();
};