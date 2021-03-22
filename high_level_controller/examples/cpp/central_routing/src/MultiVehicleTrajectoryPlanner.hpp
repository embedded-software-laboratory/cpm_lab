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

#include <memory>
#include <mutex>
#include <thread>
#include "VehicleCommandTrajectory.hpp"
#include "VehicleTrajectoryPlanningState.hpp"
using std::vector;

/**
 * \class MultiVehicleTrajectoryPlanner
 * \brief TODO
 * \ingroup central_routing
 */
class MultiVehicleTrajectoryPlanner
{
    //! TODO
    std::map<uint8_t, std::shared_ptr<VehicleTrajectoryPlanningState> > trajectoryPlans;
    //! TODO
    bool started = false;
    //! TODO
    uint64_t t_start = 0;
    //! TODO
    uint64_t t_real_time = 0;
    //! TODO
    std::mutex mutex;
    //! TODO
    std::thread planning_thread;
    //! TODO
    const uint64_t dt_nanos;
    //! TODO
    std::map<uint8_t, std::vector<TrajectoryPoint> > trajectory_point_buffer;

public:
    /**
     * \brief Constructor TODO
     * \param dt_nanos TODO
     */
    MultiVehicleTrajectoryPlanner(uint64_t dt_nanos);

    /**
     * \brief Destructor
     */
    ~MultiVehicleTrajectoryPlanner();

    /**
     * \brief TODO
     * \param t_now TODO
     */
    std::vector<VehicleCommandTrajectory> get_trajectory_commands(uint64_t t_now);

    /**
     * \brief TODO
     * \param t TODO
     */
    void set_real_time(uint64_t t);

    /**
     * \brief TODO
     */
    bool is_started() {return started;}

    /**
     * \brief TODO
     * \param vehicle TODO
     */
    void add_vehicle(std::shared_ptr<VehicleTrajectoryPlanningState> vehicle);

    /**
     * \brief TODO
     */
    void start();

};
