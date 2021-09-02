#pragma once
#include <vector>
#include <cmath>

/**
 * \class VehicleModel
 * \brief TODO
 * \ingroup vehicle
 */
class VehicleModel
{
    public:
        /**
         * \brief TODO
         * \param dynamics_parameters TODO
         * \param dt TODO
         * \param motor_throttle TODO
         * \param steering_servo TODO
         * \param battery_voltage TODO
         * \param px TODO
         * \param py TODO
         * \param yaw TODO
         * \param speed TODO
         */
        static void step(
            const std::vector<double> &dynamics_parameters,
            const double dt,
            const double motor_throttle,
            const double steering_servo,
            const double battery_voltage,
            double &px, double &py, double &yaw, double &speed);

        /**
         * \brief TODO
         * \param dynamics_parameters TODO
         * \param dt TODO
         * \param motor_throttle TODO
         * \param steering_servo TODO
         * \param battery_voltage TODO
         * \param px TODO
         * \param py TODO
         * \param yaw TODO
         * \param speed TODO
         * \param d_px TODO
         * \param d_py TODO
         * \param d_yaw TODO
         * \param d_speed TODO
         */
        static void step(
            const std::vector<double> &dynamics_parameters,
            const double dt,
            const double motor_throttle,
            const double steering_servo,
            const double battery_voltage,
            double &px, double &py, double &yaw, double &speed,
            double &d_px, double &d_py, double &d_yaw, double &d_speed);

    private:
        /**
         * \brief TODO
         * \param motor_throttle TODO
         * \param steering_servo TODO
         * \param speed TODO
         * \param d_speed TODO
         * \param d_yaw TODO
         */
        static void log_sim_warnings(
            const double motor_throttle,
            const double steering_servo,
            const double speed,
            const double d_speed,
            const double d_yaw);
};
