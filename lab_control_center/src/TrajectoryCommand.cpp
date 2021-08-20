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

#include "TrajectoryCommand.hpp"

/**
 * \file TrajectoryCommand.cpp
 * \ingroup lcc
 */

/**
 * \brief TODO
 * \ingroup lcc
 */
const uint64_t dt_nanos = 100000000ull;

TrajectoryCommand::TrajectoryCommand()
: 
    writer_vehicleCommandTrajectory("vehicleCommandTrajectory")
{
    timer = std::make_shared<cpm::TimerFD>("LabControlCenter_TrajectoryCommand", dt_nanos, 0, false);

    timer->start_async([this](uint64_t t_now){
        send_trajectory(t_now);
    },
    [](){
        //Empty lambda callback for stop signals -> Do nothing when a stop signal is received
    });
}


TrajectoryCommand::~TrajectoryCommand()
{
    std::cout << "!!! --- TrajectoryCommand destructor" << std::endl;
    timer->stop();
}

/**
 * \brief Computes the Euclidian length of a 2D vector
 * \param x x coordinate of the vector
 * \param y y coordinate of the vector
 * \ingroup lcc
 */
inline double vector_length(double x, double y)
{
    return sqrt(x*x+y*y);
}

void TrajectoryCommand::set_path(uint8_t vehicle_id, std::vector<Pose2D> path)
{
    if(path.size() < 3) return;
    if(timer == nullptr) return;

    /** Generate trajectory from given path **/
    std::vector<double> arc_length(path.size(), 0.0);

    for (size_t i = 1; i < path.size(); ++i)
    {
        const double dx = path.at(i).x() - path.at(i-1).x();
        const double dy = path.at(i).y() - path.at(i-1).y();
        arc_length.at(i) = arc_length.at(i-1) + vector_length(dx,dy);
    }

    double max_speed = 1.2;
    const double max_acceleration = 2;
    const double max_deceleration = 0.4;

    const double standstill_time = 1.5; // time [sec] at zero speed at the beginning and end
    double acceleration_time = max_speed / max_acceleration;
    double deceleration_time = max_speed / max_deceleration;

    double acceleration_distance = 0.5 * max_speed * acceleration_time;
    double deceleration_distance = 0.5 * max_speed * deceleration_time;
    const double total_distance = arc_length.back();
    double cruise_distance = total_distance - acceleration_distance - deceleration_distance;

    if(cruise_distance <= 0) 
    {
        cruise_distance = 0;

        max_speed = sqrt((2 * total_distance)/
            (1.0/max_acceleration + 1.0/max_deceleration));

        acceleration_time = max_speed / max_acceleration;
        deceleration_time = max_speed / max_deceleration;

        acceleration_distance = 0.5 * max_speed * acceleration_time;
        deceleration_distance = 0.5 * max_speed * deceleration_time;
    }

    const double cruise_time = cruise_distance / max_speed;
    const double total_time = standstill_time + acceleration_time + cruise_time + deceleration_time + standstill_time;

    const uint64_t t_start = timer->get_time() + 1000000000ull;

    vector<TrajectoryPoint> trajectory;
    for (uint64_t t_nanos = 0; (t_nanos * 1e-9) < total_time; t_nanos += dt_nanos)
    {
        double t_sec = t_nanos * 1e-9;

        double node_distance = 0;
        double node_speed = 0;


        t_sec -= standstill_time;

        if(t_sec > 0)
        {
            const double dt = fmin(t_sec, acceleration_time);
            node_distance = 0.5 * dt * dt * max_acceleration;
            node_speed = dt * max_acceleration;
        }

        t_sec -= acceleration_time;

        if(t_sec > 0)
        {
            const double dt = fmin(t_sec, cruise_time);
            node_distance += max_speed * dt;
            node_speed = max_speed;
        }

        t_sec -= cruise_time;

        if(t_sec > 0)
        {
            const double dt = fmin(t_sec, deceleration_time);
            node_speed = max_speed - dt * max_deceleration;
            node_distance += 0.5 * dt * (node_speed + max_speed);
        }

        t_sec -= deceleration_time;


        if(t_sec > 0)
        {
            node_speed = 0;
            node_distance = total_distance;
        }


        // find path index based on distance
        size_t path_index = 0;
        for (size_t i = 1; i < path.size(); ++i)
        {
            if(fabs(arc_length.at(i) - node_distance) < fabs(arc_length.at(path_index) - node_distance))
            {
                path_index = i;
            }
        }

        TrajectoryPoint p;
        p.px(path.at(path_index).x());
        p.py(path.at(path_index).y());
        p.vx(node_speed * cos(path.at(path_index).yaw()));
        p.vy(node_speed * sin(path.at(path_index).yaw()));
        p.t().nanoseconds(t_start + t_nanos);
        trajectory.push_back(p);
    }

    std::lock_guard<std::mutex> lock(_mutex);
    this->vehicle_trajectories[vehicle_id] = trajectory;
}


void TrajectoryCommand::stop(uint8_t vehicle_id)
{
    std::lock_guard<std::mutex> lock(_mutex);
    vehicle_trajectories.erase(vehicle_id);
}


void TrajectoryCommand::stop_all()
{
    std::lock_guard<std::mutex> lock(_mutex);
    vehicle_trajectories.clear();
}




void TrajectoryCommand::send_trajectory(uint64_t t_now)
{
    std::lock_guard<std::mutex> lock(_mutex);

    for(const auto& entry : vehicle_trajectories) {
        const auto vehicle_id = entry.first;
        const auto& trajectory = entry.second;

        uint64_t const max_comm_delay = 200000000ull;
        uint64_t const t_valid_nanos = t_now + max_comm_delay + dt_nanos;
        uint64_t const t_command_timeout = 500000000ull;
        

        // check for sufficient lead time. A generous value is used here
        uint64_t const t_min_lead = 500000000ull; 
        if ((!trajectory.empty()) && (trajectory.back().t().nanoseconds() < t_valid_nanos + t_min_lead + t_command_timeout)) continue;

        // don't send if trajectory starts too far in the future
        if ((!trajectory.empty()) && (trajectory.front().t().nanoseconds() > t_valid_nanos)) continue;

        // find active trajectory point
        size_t trajectory_index = 0;
        bool trajectory_found = false; //One could also use another type for the index and start with -1, but I did not prefer the necessary typecasts over using a boolean instead
        for (size_t i = 0; i < trajectory.size(); ++i) 
        {
            if(t_valid_nanos >= trajectory.at(i).t().nanoseconds())
            {
                trajectory_index = i;
                trajectory_found = true;
            }
            else if (trajectory_found)
            {
                break;
            }
        }

        if (trajectory_found)
        {
            //For interpolation: Create trajectory that starts before and ends after the found point (if possible)
            std::vector<TrajectoryPoint> trajectory_points;
            for (size_t i = trajectory_index; i < trajectory.size() && i < trajectory_index + 20; ++i)
            {
                //+20 because we cannot add too many points, else RTI causes a crash
                trajectory_points.push_back(trajectory.at(i));
            }

            VehicleCommandTrajectory command;
            command.vehicle_id(vehicle_id);
            command.trajectory_points(rti::core::vector<TrajectoryPoint>(trajectory_points));
            command.header().create_stamp().nanoseconds(t_now);
            command.header().valid_after_stamp().nanoseconds(t_valid_nanos);
            writer_vehicleCommandTrajectory.write(command);
        }
    }
}