#include "VehicleModel.hpp"

void VehicleModel::step(
    const std::vector<double> &dynamics_parameters,
    const double dt,
    const double motor_throttle,
    const double steering_servo,
    const double battery_voltage,
    double &px, double &py, double &yaw, double &speed)
{
    double d_px, d_py, d_yaw, d_speed;
    step(
        dynamics_parameters,
        dt,
        motor_throttle,
        steering_servo,
        battery_voltage,
        px, py, yaw, speed,
        d_px, d_py, d_yaw, d_speed
    );
}


void VehicleModel::step(
    const std::vector<double> &dynamics_parameters,
    const double dt,
    const double motor_throttle,
    const double steering_servo,
    const double battery_voltage,
    double &px, double &py, double &yaw, double &speed,
    double &d_px, double &d_py, double &d_yaw, double &d_speed)
{
    const auto &p = dynamics_parameters;
    const double delta = steering_servo + p[9-1];
    const double f = motor_throttle;

    d_px = p[1-1] * speed * (1 + p[2-1] * delta*delta) * cos(yaw + p[3-1] * delta + p[10-1]);
    d_py = p[1-1] * speed * (1 + p[2-1] * delta*delta) * sin(yaw + p[3-1] * delta + p[10-1]);
    d_yaw = p[4-1] * speed * delta;
    d_speed = p[5-1] * speed + (p[6-1] + p[7-1] * battery_voltage) * ((f>=0)?(1.0):(-1.0)) * pow(fabs(f), p[8-1]);

    px    += dt * d_px;
    py    += dt * d_py;
    yaw   += dt * d_yaw;
    speed += dt * d_speed;

    log_sim_warnings(motor_throttle, steering_servo, speed, d_speed, d_yaw);
        
    return;
}


void VehicleModel::log_sim_warnings(
    const double motor_throttle,
    const double steering_servo,
    const double speed,
    const double d_speed,
    const double d_yaw)
{
    // Log warnings if model is not valid
    double motor_throttle_lim = 1.0;
    double steering_servo_lim = 1.0;
    // approx. a = mu*g
    double a_lon_lim = 0.75*9.81;
    double a_lat_lim = 0.75*9.81;
    // input for motor and steering
    if (fabs(motor_throttle) > motor_throttle_lim)
    {
        cpm::Logging::Instance().write(
            "Absolute of motor_throttle=%f exceeds limit of %f.", 
            motor_throttle, motor_throttle_lim);     
    }
    if (fabs(steering_servo) > steering_servo_lim)
    {
        cpm::Logging::Instance().write(
            "Absolute of steering_servo=%f exceeds limit of %f.", 
            steering_servo, steering_servo_lim);     
    }
    // longitudinal and lateral acceleration
    if (fabs(d_speed) > a_lon_lim)
    {
        cpm::Logging::Instance().write(
            "Absolute of longitudinal acceleration %f exceeds limit of %f.", 
            d_speed, a_lon_lim);
    }
    double a_lat = speed*d_yaw;
    if (fabs(a_lat) > a_lat_lim)
    {
        cpm::Logging::Instance().write(
            "Absolute of lateral acceleration %f exceeds limit of %f.", 
            a_lat, a_lat_lim);
    }
}