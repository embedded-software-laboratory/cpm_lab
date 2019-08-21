#pragma once
#include <vector>
#include <cmath>

class VehicleModel
{
    public:
        static void step(
            const std::vector<double> &dynamics_parameters,
            const double &dt,
            const double &motor_throttle,
            const double &steering_servo,
            const double &battery_voltage,
            double &px, double &py, double &yaw, double &speed);

        static void step(
            const std::vector<double> &dynamics_parameters,
            const double &dt,
            const double &motor_throttle,
            const double &steering_servo,
            const double &battery_voltage,
            double &px, double &py, double &yaw, double &speed,
            double &d_px, double &d_py, double &d_yaw, double &d_speed);
};
