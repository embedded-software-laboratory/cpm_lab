#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "TimeSeries.hpp"
#include "defaults.hpp"
#include "cpm/Timer.hpp"

using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;

class MonitoringUi
{
public:
    Gtk::Window* window;
    Gtk::Grid* grid_vehicle_monitor;
    std::function<VehicleData()> get_vehicle_data;
    Glib::Dispatcher update_dispatcher;
    shared_ptr<cpm::Timer> update_loop;

    const vector<string> rows = { "battery_voltage", "clock_delta", "pose_x", "pose_y", "pose_yaw", "odometer_distance", "imu_acceleration_forward", "imu_acceleration_left", "speed", "motor_current" };
    
public:
    explicit MonitoringUi(std::function<VehicleData()> get_vehicle_data_callback);
    Gtk::Grid* get_parent();
};