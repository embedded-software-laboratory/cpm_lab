#include "TrajectoryInterpolation.hpp"
#include <cmath>

TrajectoryInterpolation::TrajectoryInterpolation(uint64_t stamp_now, TrajectoryPoint start_point, TrajectoryPoint end_point) 
{

    const double t_start = double(start_point.t().nanoseconds()) / 1e9;
    const double t_end = double(end_point.t().nanoseconds()) / 1e9;
    t_now = double(stamp_now) / 1e9;

    const double delta_t = t_end - t_start;
    const double tau = (t_now - t_start) / delta_t;
    
    const double tau2 = tau * tau;
    const double tau3 = tau * tau2;
    
    const double position_start_x = start_point.px();
    const double position_start_y = start_point.py();
    const double position_end_x = end_point.px();
    const double position_end_y = end_point.py();
    
    const double velocity_start_x = start_point.vx() * delta_t;
    const double velocity_start_y = start_point.vy() * delta_t;
    const double velocity_end_x = end_point.vx() * delta_t;
    const double velocity_end_y = end_point.vy() * delta_t;
    
    
    // Hermite spline coefficients
    const double p0 = 2*tau3 - 3*tau2 + 1;
    const double m0 = tau3 - 2*tau2 + tau;
    const double p1 = -2*tau3 + 3*tau2;
    const double m1 = tau3 - tau2;
    
    // Hermite spline derivative coefficients
    const double dp0 = 6*tau2 - 6*tau;
    const double dm0 = 3*tau2 - 4*tau + 1;
    const double dp1 = -6*tau2 + 6*tau;
    const double dm1 = 3*tau2 - 2*tau;
    
    // Hermite spline second derivative coefficients
    const double ddp0 = 12*tau - 6;
    const double ddm0 = 6*tau - 4;
    const double ddp1 = -12*tau + 6;
    const double ddm1 = 6*tau - 2;    
    
    position_x     =  position_start_x *   p0 + velocity_start_x *   m0 + position_end_x *   p1 + velocity_end_x *   m1;
    position_y     =  position_start_y *   p0 + velocity_start_y *   m0 + position_end_y *   p1 + velocity_end_y *   m1;
    velocity_x     = (position_start_x *  dp0 + velocity_start_x *  dm0 + position_end_x *  dp1 + velocity_end_x *  dm1) / delta_t;
    velocity_y     = (position_start_y *  dp0 + velocity_start_y *  dm0 + position_end_y *  dp1 + velocity_end_y *  dm1) / delta_t;
    acceleration_x = (position_start_x * ddp0 + velocity_start_x * ddm0 + position_end_x * ddp1 + velocity_end_x * ddm1) / (delta_t*delta_t);
    acceleration_y = (position_start_y * ddp0 + velocity_start_y * ddm0 + position_end_y * ddp1 + velocity_end_y * ddm1) / (delta_t*delta_t);
    
    yaw = atan2(velocity_y, velocity_x);
    speed = sqrt(velocity_x*velocity_x + velocity_y*velocity_y);
    curvature = (velocity_x * acceleration_y - velocity_y * acceleration_x) / (speed*speed*speed);
}