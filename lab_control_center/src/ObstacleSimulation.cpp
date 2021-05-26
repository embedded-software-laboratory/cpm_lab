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

#include "ObstacleSimulation.hpp"

/**
 * \file ObstacleSimulation.cpp
 * \ingroup lcc
 */

ObstacleSimulation::ObstacleSimulation(ObstacleSimulationData _trajectory, int _id)
:
trajectory(_trajectory)
{
    obstacle_id = static_cast<uint8_t>(_id % 256);

    //We do not accept empty trajectories
    assert(trajectory.trajectory.size() > 0);
}

CommonroadObstacle ObstacleSimulation::construct_obstacle(ObstacleSimulationSegment& point, double x, double y, double yaw, uint64_t t_now)
{
    //Create current obstacle from current time, get current trajectory point
    CommonroadObstacle obstacle;
    obstacle.vehicle_id(obstacle_id);

    //Set header
    Header header;
    header.create_stamp(TimeStamp(t_now));
    header.valid_after_stamp(TimeStamp(t_now));
    obstacle.header(header);

    //Set shape (Can be overriden by trajectory shape if required)
    obstacle.shape(point.shape);    

    //Set pose
    Pose2D pose;
    pose.x(x);
    pose.y(y);
    pose.yaw(yaw);
    obstacle.pose(pose);

    //Set velocity, if it exists
    if(point.velocity.has_value())
    {
        obstacle.speed(point.velocity.value().get_mean());
    }

    //Set further obstacle information
    obstacle.pose_is_exact(point.is_exact);        
    obstacle.is_moving((trajectory.trajectory.size() > 1));
    obstacle.type(trajectory.obstacle_type);
    obstacle.obstacle_class(trajectory.obstacle_class);

    return obstacle;
}

VehicleCommandTrajectory ObstacleSimulation::construct_trajectory(std::vector<TrajectoryPoint>& trajectory_points, uint64_t t_now)
{
    VehicleCommandTrajectory trajectory;
    Header header;
    header.create_stamp(TimeStamp(t_now));
    header.valid_after_stamp(TimeStamp(t_now));
    trajectory.header(header);
    trajectory.trajectory_points(trajectory_points);
    trajectory.vehicle_id(obstacle_id);

    return trajectory;
}

std::pair<double, double> ObstacleSimulation::get_position(ObstacleSimulationSegment& segment)
{
    std::pair<double, double> position(0, 0);

    //Get position value
    if (segment.position.has_value())
    {
        position = segment.position.value();
    }

    //Add mean of shape position (lanelet ref is already translated to position in ObstacleSimulationManager)
    double x, y = 0.0;
    double center_count = 0.0;

    for (auto circle : segment.shape.circles())
    {
        auto center = circle.center();
        x += center.x();
        y += center.y();
        ++center_count;
    }

    for (auto polygon : segment.shape.polygons())
    {
        double sum_x = 0;
        double sum_y = 0;

        for (auto point : polygon.points())
        {
            sum_x += point.x();
            sum_y += point.y();
        }

        x += sum_x / polygon.points().size();
        y += sum_y / polygon.points().size();
        ++center_count;
    }

    for (auto rectangle : segment.shape.rectangles())
    {
        auto center = rectangle.center();
        x += center.x();
        y += center.y();
        ++center_count;
    }

    if (center_count > 0)
    {
        x /= center_count;
        y /= center_count;
    }

    position.first += x;
    position.second += y;

    return position;
}

CommonroadObstacle ObstacleSimulation::get_init_state(uint64_t t_now)
{
    auto& point = trajectory.trajectory.at(0);

    //These values might not exist
    double x = 0;
    double y = 0;
    double yaw = 0;
    if(point.position.has_value())
    {
        x = point.position.value().first;
        y = point.position.value().second;
        yaw = point.orientation.value_or(0.0);
    }
    else
    {
        yaw = point.orientation.value_or(0.0);
        //In this case, the position is given by either the shape's values or by a lanelet reference (which was transformed to a shape by the simulation manager before then)
    }

    return construct_obstacle(point, x, y, yaw, t_now);
}

void ObstacleSimulation::interpolate_between(ObstacleSimulationSegment& p1, ObstacleSimulationSegment& p2, double current_time, double time_step_size, double &x_interp, double &y_interp, double &yaw_interp)
{
    //TODO: Find better interpolation, this one does not work
    // if (p1.velocity.has_value() && p2.velocity.has_value())
    // {
    //     //Like Janis Interpolation, but with other datatypes, thus copied and modified
    //     const double t_start = p1.time.value().get_mean() * time_step_size / 1e9;
    //     const double t_end = p2.time.value().get_mean() * time_step_size / 1e9;
    //     double t_now = current_time / 1e9;

    //     const double delta_t = t_end - t_start;
    //     const double tau = (t_now - t_start) / delta_t;
        
    //     const double tau2 = tau * tau;
    //     const double tau3 = tau * tau2;
        
    //     const double position_start_x = p1.position.first;
    //     const double position_start_y = p1.position.second;
    //     const double position_end_x = p2.position.first;
    //     const double position_end_y = p2.position.second;

    //     //Get velocity x, y - expectation: orientation value gives velocity orientation, not position values
    //     double p1_velocity_x = std::cos(p1.orientation.value_or(0.0)) * p1.velocity.value().get_mean();
    //     double p1_velocity_y = std::sin(p1.orientation.value_or(0.0)) * p1.velocity.value().get_mean();
    //     double p2_velocity_x = std::cos(p2.orientation.value_or(0.0)) * p2.velocity.value().get_mean();
    //     double p2_velocity_y = std::sin(p2.orientation.value_or(0.0)) * p2.velocity.value().get_mean();
        
    //     const double velocity_start_x = p1_velocity_x * delta_t;
    //     const double velocity_start_y = p1_velocity_y * delta_t;
    //     const double velocity_end_x = p2_velocity_x * delta_t;
    //     const double velocity_end_y = p2_velocity_y * delta_t;
        
        
    //     // Hermite spline coefficients
    //     const double p0 = 2*tau3 - 3*tau2 + 1;
    //     const double m0 = tau3 - 2*tau2 + tau;
    //     const double p1 = -2*tau3 + 3*tau2;
    //     const double m1 = tau3 - tau2;
        
    //     // Hermite spline derivative coefficients
    //     const double dp0 = 6*tau2 - 6*tau;
    //     const double dm0 = 3*tau2 - 4*tau + 1;
    //     const double dp1 = -6*tau2 + 6*tau;
    //     const double dm1 = 3*tau2 - 2*tau;  
        
    //     x_interp     =  position_start_x *   p0 + velocity_start_x *   m0 + position_end_x *   p1 + velocity_end_x *   m1;
    //     y_interp     =  position_start_y *   p0 + velocity_start_y *   m0 + position_end_y *   p1 + velocity_end_y *   m1;
    //     double velocity_x     = (position_start_x *  dp0 + velocity_start_x *  dm0 + position_end_x *  dp1 + velocity_end_x *  dm1) / delta_t;
    //     double velocity_y     = (position_start_y *  dp0 + velocity_start_y *  dm0 + position_end_y *  dp1 + velocity_end_y *  dm1) / delta_t;
    //     yaw_interp = std::atan2(velocity_y, velocity_x);
    // }
    // else
    // {
        //Fallback: Linear interpolation (if no velocity is set)
        double intermediate_frac = (current_time - (p1.time.value().get_mean() * time_step_size)) / ((p2.time.value().get_mean() - p1.time.value().get_mean()) * time_step_size);
        x_interp = (p2.position.value().first - p1.position.value().first) * intermediate_frac + p1.position.value().first;
        y_interp = (p2.position.value().second - p1.position.value().second) * intermediate_frac + p1.position.value().second;
        yaw_interp = (p2.orientation.value_or(0.0) - p1.orientation.value_or(0.0)) * intermediate_frac + p1.orientation.value_or(0.0);
    //}
}

CommonroadObstacle ObstacleSimulation::get_state(uint64_t start_time, uint64_t t_now, uint64_t time_step_size)
{
    //For non-dynamic obstacles: We can still use the initial state
    if (trajectory.obstacle_class != ObstacleClass::Dynamic)
    {
        return get_init_state(t_now);
    }

    //For dynamic obstacles: We must be able to use time.value(), as it is a required field
    assert(trajectory.trajectory.at(current_trajectory).time.has_value());

    //Get to currently active index / trajectory point
    while (t_now - start_time >= trajectory.trajectory.at(current_trajectory).time.value().get_mean() * time_step_size && current_trajectory < trajectory.trajectory.size() - 1)
    {
        ++current_trajectory;
    }
    auto& point = trajectory.trajectory.at(current_trajectory);

    //These values are set either by interpolation or using the last data point or do not exist
    double x = 0;
    double y = 0;
    double yaw = 0;
    if(point.position.has_value())
    {
        //Interpolate or stay at start / end point
        if (point.time.value().get_mean() * time_step_size >= t_now - start_time && current_trajectory > 0)
        {
            //Interpolate
            interpolate_between(trajectory.trajectory.at(current_trajectory - 1), point, t_now - start_time, time_step_size, x, y, yaw);
        }
        else
        {
            //Stay at start / final point
            x = point.position.value().first;
            y = point.position.value().second;
            yaw = point.orientation.value_or(0.0);
        }
    }
    else
    {
        yaw = point.orientation.value_or(0.0);

        //In this case, the position is given by either the shape's values or by a lanelet reference (which was transformed to a shape by the simulation manager before then)
    }
    
    return construct_obstacle(point, x, y, yaw, t_now);
}

VehicleCommandTrajectory ObstacleSimulation::get_init_trajectory(uint64_t t_now, uint64_t timer_step_size)
{
    std::vector<TrajectoryPoint> trajectory_points;

    assert(trajectory.trajectory.at(0).time.has_value());
    auto& first_point = trajectory.trajectory.at(0);

    for (size_t i = 0; i < 2; ++i)
    {
        TrajectoryPoint point;
        point.t(TimeStamp(t_now + i * timer_step_size));
        
        auto position = get_position(first_point);
        point.px(position.first);
        point.py(position.second);
        point.vx(0);
        point.vy(0);

        trajectory_points.push_back(point);
    }
    return construct_trajectory(trajectory_points, t_now);
}

VehicleCommandTrajectory ObstacleSimulation::get_trajectory(uint64_t start_time, uint64_t t_now, uint64_t time_step_size)
{
    std::vector<TrajectoryPoint> trajectory_points;

    //We must be able to use time.value(), as it is a required field
    assert(trajectory.trajectory.at(current_trajectory).time.has_value());

    //Get to currently active index / trajectory point
    while (t_now - start_time >= trajectory.trajectory.at(current_trajectory).time.value().get_mean() * time_step_size && current_trajectory < trajectory.trajectory.size() - 1)
    {
        ++current_trajectory;
    }

    //Behaviour for points before the final point
    if (t_now - start_time < trajectory.trajectory.at(trajectory.trajectory.size() - 1).time.value().get_mean() * time_step_size)
    {
        //Send from previous over current point up to some time steps in the future
        size_t start_index = current_trajectory;
        if (start_index > 0)
        {
            --start_index;
        }

        double previous_direction = 0.0; //For trajectory interpolation

        //Send current and future points, but do not create the final point here if that one would be reached
        for (size_t index = start_index; index < start_index + future_time_steps && index < trajectory.trajectory.size() - 2; ++index)
        {
            auto& current_point = trajectory.trajectory.at(index);

            TrajectoryPoint point;
            point.t(TimeStamp(start_time + current_point.time.value().get_mean() * time_step_size));

            auto position = get_position(current_point);
            point.px(position.first);
            point.py(position.second);

            //Get next position for interpolation, calculate angle to it (current direction) and total speed to reach the point based on dx, dy and dt
            auto next_position = get_position(trajectory.trajectory.at(index + 1));
            const double current_direction = atan2(
                next_position.second - position.second,
                next_position.first - position.first
            );

            double v_total = 0.0;
            if (! current_point.velocity.has_value())
            {
                v_total = sqrt(pow((next_position.first - position.first), 2) + pow((next_position.second - position.second), 2)) / time_step_size * 1e9;
            }
            else
            {
                v_total = current_point.velocity.value().get_mean() / 2;
            }

            //If an orientation value already exists, we do not need to interpolate that
            if (current_point.orientation.has_value())
            {
                auto yaw = current_point.orientation.value();
                point.vx(cos(yaw) * v_total);
                point.vy(sin(yaw) * v_total);

                previous_direction = yaw;
            }
            else if (index == 0)
            {
                //Different behaviour for start point: Here, the velocity can simply be calculated using the difference in position
                point.vx(cos(current_direction) * v_total);
                point.vy(sin(current_direction) * v_total);

                previous_direction = current_direction;
            }
            else
            {
                auto previous_position = get_position(trajectory.trajectory.at(index - 1));
                
                //Calculate angles regarding dx and dy to the distance dp between points: Between the previous and current and the current and next point
                //Use angle:            |
                // alpha                | dy
                // _____________________|
                //           dx
                //to get from total speed set to vx and vz
                // double alpha = 0.0;
                // double v_total = 0.0;

                // if (! current_point.velocity.has_value())
                // {
                //     v_total = sqrt(pow((next_position.first - position.first), 2) + pow((next_position.second - position.second), 2));
                // }
                // else
                // {
                //     v_total = current_point.velocity.value().get_mean();
                // }

                // //Angle to next point
                // alpha = atan((next_position.second - position.second) / (next_position.first - position.first)); //alpha = arctan(dy / dx)

                // //Calculate alpha from both the previous and next or only the next point
                // if (index > 0)
                // {
                //     auto prev_position = get_position(trajectory.trajectory.at(index + 1));
                //     alpha += atan((position.second - prev_position.second) / (position.first - prev_position.first));
                //     alpha /= 2.0;
                // }
                
                // //Calculate speed parts from angle
                // point.vx(cos(alpha) * v_total);
                // point.vy(sin(alpha) * v_total);

                //Very basic method
                // point.vx((next_position.first - position.first) / time_step_size);
                // point.vy((next_position.second - position.second) / time_step_size);

                //Janis' method
                const double current_direction = atan2(
                    position.second - previous_position.second,
                    position.first - previous_position.first
                );

                double delta_yaw = current_direction - previous_direction;
                delta_yaw = remainder(delta_yaw, 2*M_PI);

                double new_direction = remainder(previous_direction+delta_yaw, 2*M_PI);
                point.vx(cos(new_direction) * v_total);
                point.vy(sin(new_direction) * v_total);

                previous_direction = new_direction;
            }
            
            trajectory_points.push_back(point);
        }
    }

    //If we have reached the final point, send that point (multiple times)
    if (trajectory_points.size() < future_time_steps)
    {
        auto& last_point = trajectory.trajectory.at(trajectory.trajectory.size() - 1);
        for (int i = trajectory_points.size(); i < 4; ++i)
        {
            TrajectoryPoint point;
            point.t(TimeStamp(t_now + i * time_step_size));
            
            auto position = get_position(last_point);
            point.px(position.first);
            point.py(position.second);
            point.vx(0);
            point.vy(0);

            trajectory_points.push_back(point);
        }
    }

    return construct_trajectory(trajectory_points, t_now);
}

uint8_t ObstacleSimulation::get_id()
{
    return obstacle_id;
}

void ObstacleSimulation::reset()
{
    current_trajectory = 0;
}