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

#include "MultiVehicleTrajectoryPlanner.hpp"

#define T_START_DELAY_NANOS 5000000000ull
#define T_PLAN_AHEAD_NANOS 6000000000ull

/**
 * \file MultiVehicleTrajectoryPlanner.cpp
 * \ingroup central_routing
 */

MultiVehicleTrajectoryPlanner::MultiVehicleTrajectoryPlanner(uint64_t dt_nanos):dt_nanos(dt_nanos){}

MultiVehicleTrajectoryPlanner::~MultiVehicleTrajectoryPlanner(){
    if ( planning_thread.joinable() ) planning_thread.join();
}

std::vector<VehicleCommandTrajectory> MultiVehicleTrajectoryPlanner::get_trajectory_commands(uint64_t t_now)
{
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<VehicleCommandTrajectory> result;
    for(auto &e:trajectory_point_buffer)
    {
        VehicleCommandTrajectory vehicleCommandTrajectory;
        vehicleCommandTrajectory.vehicle_id(e.first);
        vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(e.second));
        vehicleCommandTrajectory.header().create_stamp().nanoseconds(t_now); //You just need to set t_now here, as it was created at t_now
        vehicleCommandTrajectory.header().valid_after_stamp().nanoseconds(t_now + 1000000000ull); //Hardcoded value from the planner (t_start), should be correct (this value should correlate with the trajectory point that should be valid at t_now)
        result.push_back(vehicleCommandTrajectory);
    }
    return result;
}

void MultiVehicleTrajectoryPlanner::set_real_time(uint64_t t)
{
    std::lock_guard<std::mutex> lock(mutex);
    t_real_time = t;
}


void MultiVehicleTrajectoryPlanner::add_vehicle(std::shared_ptr<VehicleTrajectoryPlanningState> vehicle)
{   //fill vector with future trajectory points of other vehicles
    assert(!started);
    trajectoryPlans[vehicle->get_vehicle_id()] = vehicle;
}

void MultiVehicleTrajectoryPlanner::start()
{
    assert(!started);
    started = true;

    planning_thread = std::thread([this](){
        uint64_t t_planning = 0;

        {
            std::lock_guard<std::mutex> lock(mutex); 
            t_planning = t_real_time;
        }

        while(1)
        {
            // Priority based collision avoidance: Every vehicle avoids 
            // the 'previous' vehicles, in this example those with a smaller ID.
            vector< std::shared_ptr<VehicleTrajectoryPlanningState> > previous_vehicles;
            bool is_collision_avoidable = false;
            for(auto &e:trajectoryPlans)
            {
                is_collision_avoidable = e.second->avoid_collisions(previous_vehicles);
                if (!is_collision_avoidable){
                    break;
                }
                previous_vehicles.push_back(e.second);
            }

            if (!is_collision_avoidable){
                started = false; // end planning
                break;
            }                

            {
                std::lock_guard<std::mutex> lock(mutex); 

                if(t_start == 0)
                {
                    t_start = t_real_time + T_START_DELAY_NANOS;
                    for(auto &e:trajectoryPlans)
                    {
                        auto trajectory_point = e.second->get_trajectory_point();
                        trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_real_time);
                        trajectory_point_buffer[e.first].push_back(trajectory_point);
                    }
                }

                for(auto &e:trajectoryPlans)
                {
                    // delete all points that are no longer relevant
                    // find latest point in the past
                    std::vector<TrajectoryPoint>::iterator it_to_delete = trajectory_point_buffer[e.first].begin();
                    assert(!trajectory_point_buffer[e.first].empty());
                    for (std::vector<TrajectoryPoint>::iterator tp_it = trajectory_point_buffer[e.first].begin() + 1; tp_it != trajectory_point_buffer[e.first].end(); ++tp_it)
                    {
                        if (tp_it->t().nanoseconds() > t_real_time)
                        {
                            it_to_delete = tp_it-1;
                            break;
                        }
                    }
                    trajectory_point_buffer[e.first].erase(trajectory_point_buffer[e.first].begin(), it_to_delete);

                    auto trajectory_point = e.second->get_trajectory_point();
                    trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_start);
                    trajectory_point_buffer[e.first].push_back(trajectory_point);
                }
            }

            for(auto &e:trajectoryPlans)
            {
                e.second->apply_timestep(dt_nanos);
            }

            t_planning += dt_nanos;

            while(t_real_time + T_PLAN_AHEAD_NANOS < t_planning) usleep(110000);
        }
    });

}