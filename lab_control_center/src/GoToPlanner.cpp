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

#include "GoToPlanner.hpp"

#include <algorithm> // std::max
#include <cmath>    // M_PI
#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "MatlabEngine.hpp"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include "MatlabDataArray.hpp"
#pragma GCC diagnostic pop

using namespace matlab::engine;

GoToPlanner::GoToPlanner(
    std::function<std::vector<Pose2D>()> get_goal_poses
    ,std::function<VehicleData()> get_vehicle_data
    ,std::shared_ptr<TrajectoryCommand> trajectory_command
)
:get_goal_poses(get_goal_poses)
,get_vehicle_data(get_vehicle_data)
,trajectory_command(trajectory_command)
{
}


// TODO remove vehicle_ids, get from timeseriesaggregator
void GoToPlanner::go_to_start_poses()
{
    // get start poses from CommonRoadScenario
    std::vector<Pose2D> goal_poses = get_goal_poses();
    go_to_poses(goal_poses);
    return;
}

void GoToPlanner::go_to_poses(
    std::vector<Pose2D> goal_poses
)
{
    std::cout << "Going to poses ..." << std::endl;
    
    if (planner_thread.joinable())
    {
        // end running thread
        planner_thread.join();
    }

    planner_thread = std::thread([=]()
    {

        // Start MATLAB engine synchronously
        std::unique_ptr<MATLABEngine> matlabPtr = connectMATLAB();

        // Create MATLAB data array factory
        matlab::data::ArrayFactory factory;

        // TODO use absolute_executable_path
        // add matlab functions path
        matlabPtr->eval(u"addpath('../tools/go_to_formation/matlab/');");

        // locate all vehicles
        std::vector<double> vehicle_poses; // [m] [m] [deg]!
        std::vector<uint8_t> vehicle_ids;
        VehicleData vehicle_data = get_vehicle_data();
        for(const auto& entry : vehicle_data) {
            const auto vehicle_id = entry.first;
            const auto& vehicle_timeseries = entry.second;

            // if(!vehicle_timeseries.at("pose_x")->has_new_data(1.0)) continue;
            double x = vehicle_timeseries.at("pose_x")->get_latest_value();
            double y = vehicle_timeseries.at("pose_y")->get_latest_value();
            double yaw = vehicle_timeseries.at("pose_yaw")->get_latest_value();
            vehicle_poses.push_back(x);
            vehicle_poses.push_back(y);
            vehicle_poses.push_back(yaw*180.0/M_PI);

            vehicle_ids.push_back(vehicle_id);
        }
        // Plan path for every vehicle
        std::size_t nVeh = vehicle_data.size();
        // TODO go to poses that are available
        if (goal_poses.size() != nVeh)
        {
            std::cerr   << "Number of goal poses " << goal_poses.size()
                        << "!= number of vehicles " << nVeh << std::endl;
            return;
        }
        std::vector<int> veh_at_goal(nVeh, 0);
        bool is_vehicle_moveable = true;
        uint64_t planning_delay = 1000000000ull;
        uint64_t last_traj_duration = 0ull;
        while ( is_vehicle_moveable )
        {
            is_vehicle_moveable = false;
            for (std::size_t iVeh = 0; iVeh < nVeh; ++iVeh)
            {
                if (veh_at_goal[iVeh]) continue;

                // Plan path in MATLAB
                std::vector<matlab::data::Array> matlab_args({
                    // vehicle poses
                    factory.createArray<double>(
                        {3, nVeh},
                        vehicle_poses.data(),
                        vehicle_poses.data()+3*nVeh
                    ),
                    // vehicle index
                    factory.createScalar<int>(iVeh+1),
                    // goal pose
                    factory.createArray<double>(
                        {3, 1},
                        {
                            goal_poses[iVeh].x(),
                            goal_poses[iVeh].y(),
                            goal_poses[iVeh].yaw()*180.0/M_PI
                        }
                    )
                });
                
                std::vector<matlab::data::Array> result = matlabPtr->feval(
                    u"plan_path", 2, matlab_args
                );

                matlab::data::ArrayDimensions path_dims = result[0].getDimensions();
                matlab::data::Array is_valid = result[1];

                if (int16_t(is_valid[0]) == 0)
                {
                    std::cout   << "Found no valid path for vehicle "
                                << static_cast<int>(vehicle_ids[iVeh]) << std::endl;
                    continue;
                }
                is_vehicle_moveable = true;

                // pass path to trajectory_command
                std::vector<Pose2D> path_pts;
                for (std::size_t i_path_pt = 0; i_path_pt < path_dims[1]; ++i_path_pt)
                {
                    Pose2D path_point;
                    path_point.x(double(result[0][0][i_path_pt]));
                    path_point.y(double(result[0][1][i_path_pt]));
                    path_point.yaw(M_PI/180.0*double(result[0][2][i_path_pt]));
                    path_pts.push_back(path_point);
                }
                uint64_t new_traj_duration = trajectory_command->set_path(
                    vehicle_ids[iVeh],
                    path_pts,
                    std::max<uint64_t>(last_traj_duration, planning_delay)
                );
                last_traj_duration = new_traj_duration;
                veh_at_goal[iVeh] = 1;
                
                // assume vehicle at goal pose
                vehicle_poses.at(iVeh*3+0) = path_pts.back().x();
                vehicle_poses.at(iVeh*3+1) = path_pts.back().y();
                vehicle_poses.at(iVeh*3+2) = path_pts.back().yaw()*180.0/M_PI;
                std::cout   << "Planned trajectory for vehicle "
                            << static_cast<int>(vehicle_ids[iVeh]) << std::endl;
                break;
            }
        }
        std::cout   << "Going to poses done." << std::endl;
    }  // thread lambda function
    ); // thread call
}