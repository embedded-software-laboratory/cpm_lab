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
#include <stdint.h>
#include "defaults.hpp"
#include "Pose2D.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/TimerFD.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"

/**
 * \class TrajectoryCommand
 * \brief Using this class, trajectories from 2D paths can be created and sent to the vehicles
 */
class TrajectoryCommand
{
    std::mutex _mutex;
    map<uint8_t, vector<TrajectoryPoint>> vehicle_trajectories;
    std::shared_ptr<cpm::TimerFD> timer;

    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory;


    void send_trajectory(uint64_t t_now);

public:
    TrajectoryCommand();
    ~TrajectoryCommand();
    void set_path(uint8_t vehicle_id, std::vector<Pose2D> path);
    void stop(uint8_t vehicle_id);
    void stop_all();
    
};