#pragma once
#include <vector>
#include <cmath>
#include <cpm/Logging.hpp>

class VehicleModel
{
    public:
        static void step(
            const std::vector<double> &dynamics_parameters,
            const double dt,
            const double motor_throttle,
            const double steering_servo,
            const double battery_voltage,
            double &px, double &py, double &yaw, double &speed);

        static void step(
            const std::vector<double> &dynamics_parameters,
            const double dt,
            const double motor_throttle,
            const double steering_servo,
            const double battery_voltage,
            double &px, double &py, double &yaw, double &speed,
            double &d_px, double &d_py, double &d_yaw, double &d_speed);

    private:
        static void log_sim_warnings(
            const double motor_throttle,
            const double steering_servo,
            const double speed,
            const double d_speed,
            const double d_yaw);
};
