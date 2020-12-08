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

#include "PathTrackingController.hpp"
#include <cassert>
#include <cmath>
// #include <iostream>
// #include <sstream>
// #include "cpm/Logging.hpp"
#include "PathInterpolation.hpp"

PathTrackingController::PathTrackingController(uint8_t _vehicle_id, std::function<void(double&, double&)> _stop_vehicle)
:
    writer_Visualization(
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),
        cpm::get_topic<Visualization>("visualization"),
        dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()),
    vehicle_id(vehicle_id),
    stop_vehicle(stop_vehicle)
{}


// Wrap angle in radians to [-pi pi]
inline double wrap2pi(const double yaw)
{
    double yaw_out = fmod(yaw + M_PI, 2.0*M_PI);
    if (yaw_out < 0) yaw_out += 2.0*M_PI;
    return yaw_out - M_PI;
}

double PathTrackingController::control_steering_servo(
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
    Pose2D ref_pose = find_reference_pose(path, x_f, y_f);


    // compute control errors
    double tHat_x = cos(ref_pose.yaw());
    double tHat_y = sin(ref_pose.yaw());
    double d_x = x_f - ref_pose.x();
    double d_y = y_f - ref_pose.y();
    
    double e = d_x*tHat_y - d_y*tHat_x;
    double psi = wrap2pi(ref_pose.yaw() - yaw);


    // send reference pose as virtualisation
    Visualization vis;
    vis.id(vehicle_id);
    vis.type(VisualizationType::LineStrips);
    vis.time_to_live(25000000ull);
    vis.size(0.03);
    vis.color().r(255);
    vis.color().g(0);
    vis.color().b(240);

    const double vis_length = 0.4;

    vis.points().resize(2);
    vis.points().at(0).x(ref_pose.x());
    vis.points().at(0).y(ref_pose.y());
    vis.points().at(1).x(ref_pose.x() + tHat_x * vis_length * vehicleState.speed());
    vis.points().at(1).y(ref_pose.y() + tHat_y * vis_length * vehicleState.speed());

    writer_Visualization.write(vis);


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

Pose2D find_reference_pose(
    const std::vector<PathPoint> &path,
    const double x,
    const double y
)
{
    // Iterate over interpolated path to find point with minimum distance
    const double ds = 0.01; // [m]
    assert(path.size() > 1);
    size_t i_path_point = 0;
    double s_max = path.back().s();
    double min_dist = 1e300;
    Pose2D result;
    for (double s_query = 0; s_query <= s_max; s_query += ds)
    {
        // Advance path points for interpolation if necessary
        assert( i_path_point < path.size()-1 );
        if (s_query > path[i_path_point+1].s())
        {
            ++i_path_point;
        }

        // calculate distance to reference path
        PathInterpolation path_interpolation(
            s_query, path[i_path_point], path[i_path_point+1]
        );
        double dx = x - path_interpolation.position_x;
        double dy = y - path_interpolation.position_y;
        double distance = sqrt( dx*dx + dy*dy );

        if (distance < min_dist)
        {
            min_dist = distance;
            // update reference pose
            result.x(path_interpolation.position_x);
            result.y(path_interpolation.position_y);
            result.yaw(path_interpolation.yaw);
        }
    }
    return result;
}