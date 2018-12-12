#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "TimeSeries.hpp"
#include "defaults.hpp"
#include "cpm/Timer.hpp"

class MonitoringUi
{
    Glib::RefPtr<Gtk::Builder> builder;
    Gtk::Window* window;
    Gtk::Grid* grid_vehicle_monitor;
    Glib::Dispatcher update_dispatcher;
    shared_ptr<cpm::Timer> update_loop;
    const map<uint8_t, map<string, shared_ptr<TimeSeries> > >& vehicle_data;

    const vector<string> rows = { "battery_voltage", "clock_delta", "pose_x", "pose_y", "pose_yaw", "odometer_distance", "imu_acceleration_forward", "imu_acceleration_left", "speed", "motor_current" };
    
public:
    explicit MonitoringUi(const map<uint8_t, map<string, shared_ptr<TimeSeries> > >&);
    Gtk::Window& get_window();    
};