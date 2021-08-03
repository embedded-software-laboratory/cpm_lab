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

#pragma once

#include <memory> // shared_ptr
#include <thread>
#include <vector>

#include "Pose2D.hpp"
#include "TimeSeriesAggregator.hpp"
#include "TrajectoryCommand.hpp"

/**
 * \class GoToPlanner
 * \brief Class to control vehicles to poses 
 * \ingroup lcc
 */
class GoToPlanner {
public:
    GoToPlanner(
        std::function<std::vector<Pose2D>()> get_goal_poses
        ,std::function<VehicleData()> get_vehicle_data
        ,std::shared_ptr<TrajectoryCommand> trajectory_command
        ,std::string absolute_executable_path
    );

    void go_to_start_poses();
    void go_to_poses(std::vector<Pose2D> goal_poses);

private:
    std::function<std::vector<Pose2D>()> get_goal_poses;
    std::function<VehicleData()> get_vehicle_data;
    std::shared_ptr<TrajectoryCommand> trajectory_command;
    std::thread planner_thread;
    std::string matlab_functions_path;
};