#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "TimeSeries.hpp"
#include "defaults.hpp"

using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;

class MonitoringUi
{
public:
    Gtk::Window* window;
    Gtk::Grid* grid_vehicle_monitor;
    std::function<VehicleData()> get_vehicle_data;
    
    //Before: TimerFD, but this class is stopped by stop signals which might be emitted multiple times by the LCC depending on user interaction
    //Thus: Own timer implementation instead
    void ui_update_loop();
    Glib::Dispatcher update_dispatcher; //to communicate between thread and GUI
    std::thread ui_thread;
    std::atomic_bool run_thread;

    const vector<string> rows = { "battery_voltage", "battery_level", "clock_delta", "pose_x", "pose_y", "pose_yaw", "ips_x", "ips_y", "ips_yaw", "odometer_distance", "imu_acceleration_forward", "imu_acceleration_left", "speed", "motor_current" };
    
public:
    explicit MonitoringUi(std::function<VehicleData()> get_vehicle_data_callback);
    ~MonitoringUi();
    Gtk::Grid* get_parent();
};