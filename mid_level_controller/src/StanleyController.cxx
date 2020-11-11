// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "StanleyController.hpp"
#include <cassert>
#include <cmath>
// #include <iostream>
// #include <sstream>
// #include "cpm/Logging.hpp"
// #include "TrajectoryInterpolation.hpp"

StanleyController::StanleyController(uint8_t _vehicle_id, std::function<void(double&, double&)> _stop_vehicle)
:vehicle_id(vehicle_id)
,stop_vehicle(stop_vehicle)
{
}


// Wrap angle in radians to [-pi pi]
inline double wrap2pi(const double yaw)
{
    double yaw_out = fmod(yaw + M_PI_2, M_PI);
    if (yaw_out < 0) yaw_out += M_PI;
    return yaw_out - M_PI_2;
}

double StanleyController::control_steering_servo(
        const VehicleState &vehicleState,                  
        const VehicleCommandPathTracking &commandPathTracking
)
{
    // Compute front axle position
    double x   = vehicleState.pose().x();
    double y   = vehicleState.pose().y();
    double yaw = vehicleState.pose().yaw();

    const double wheelbase = 0.15; // m
    double x_f = x + wheelbase/2*cos(yaw);
    double y_f = y + wheelbase/2*sin(yaw);

    // Find reference point on path
    std::vector<PathPoint> path = commandPathTracking.path();
    Pose2D ref_pose = find_reference_pose2d(path, x_f, y_f, yaw);


    // compute control errors
    double tHat_x = cos(ref_pose.yaw());
    double tHat_y = sin(ref_pose.yaw());
    double d_x = x_f - ref_pose.x();
    double d_y = y_f - ref_pose.y();
    
    double e = d_x*tHat_y - d_y*tHat_x;
    double psi = wrap2pi(ref_pose.yaw() - yaw);


    // control law for steering servo
    double k = 5;
    double k_soft = 0.3;
    double delta = psi 
        + atan( k*e / (k_soft+vehicleState.speed()) );
        // + k_dsteer * (delta_last(1) - delta_last(2));
        // - k_dyaw*(dyaw - path.k(i_path_ref)*vehicleState.speed()) ...
        
    // tune down aggressiveness at fast speeds
    delta = delta / ( (0.25*vehicleState.speed()*vehicleState.speed()) + 1 );
    
    double steering_servo = 0.226*(exp(3.509*delta)-exp(-3.509*delta));
    return steering_servo;
}

Pose2D StanleyController::find_reference_pose2d(
    const std::vector<PathPoint> &path,
    const double x,
    const double y,
    const double yaw
)
{
    // Iterate over interpolated path to find point with minimum distance



}