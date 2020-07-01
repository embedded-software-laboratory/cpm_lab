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

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"

#include <memory>

#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/SimpleTimer.hpp"
#include "CommonroadObstacle.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"
#include <dds/pub/ddspub.hpp>


/**
 * \class ObstacleSimulation
 * \brief Nested class responsible for simulating a single obstacle
 * Objects of this class have access to the writer / other members, e.g. the writer
 */
class ObstacleSimulation
{
private: 
    //DDS
    dds::pub::DataWriter<CommonroadObstacle> writer_commonroad_obstacle;

    //Trajectory info
    uint8_t obstacle_id;
    std::vector<CommonTrajectoryPoint> trajectory; //Important: Position should always be set! Translate lanelet refs beforehand!
    uint64_t time_step_size;
    size_t current_trajectory = 0;

    //Timing
    bool simulated_time;
    std::string node_id;
    uint64_t dt_nanos;
    uint64_t start_time;
    std::shared_ptr<cpm::Timer> simulation_timer;
    std::shared_ptr<cpm::SimpleTimer> standby_timer;

    /**
     * \brief Interpolation function that delivers state values in between set trajectory points
     * \return x,y,yaw values using references as input
     */
    void interpolate_between(CommonTrajectoryPoint p1, CommonTrajectoryPoint p2, double current_time, double &x_interp, double &y_interp, double &yaw_interp);
    
    //Send the current obstacle state based on the given trajectory point
    void send_state(CommonTrajectoryPoint& point, uint64_t t_now);

    void stop_timers();

public:
    /**
     * \brief constructor
     * \param _trajectory The trajectory to follow: Important: Translate lanelet ref to position beforehand, so that it must not be done here anymore (a value is expected for every single trajectory point)
     */
    ObstacleSimulation(std::vector<CommonTrajectoryPoint> _trajectory, double _time_step_size, int _id, bool _simulated_time);

    //Destructor for timer
    ~ObstacleSimulation();

    /**
     * \brief Send the initial state of the obstacles periodically until the simulation is started
     */
    void send_init_state();

    void start();
    void reset();
};