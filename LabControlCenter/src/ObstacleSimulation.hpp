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
#include "commonroad_classes/ObstacleSimulationData.hpp"

#include <memory>

#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/SimpleTimer.hpp"
#include "CommonroadObstacle.hpp"
#include "VehicleCommandTrajectory.hpp"
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
    //Trajectory info
    uint8_t obstacle_id;
    ObstacleSimulationData trajectory; //Important: Position should always be set! Translate lanelet refs beforehand!
    size_t current_trajectory = 0;

    const size_t future_time_steps = 10; //Send up to 10 trajectory points from future time steps

    /**
     * \brief Interpolation function that delivers state values in between set trajectory points
     * \return x,y,yaw values using references as input
     */
    void interpolate_between(ObstacleSimulationSegment& p1, ObstacleSimulationSegment& p2, double current_time, double time_step_size, double &x_interp, double &y_interp, double &yaw_interp);

    CommonroadObstacle construct_obstacle(ObstacleSimulationSegment& point, double x, double y, double yaw, uint64_t t_now);

    VehicleCommandTrajectory construct_trajectory(std::vector<TrajectoryPoint>& trajectory_points, uint64_t t_now);

    std::pair<double, double> get_position(ObstacleSimulationSegment& segment);

public:
    /**
     * \brief constructor
     * \param _trajectory The trajectory to follow: Important: Translate lanelet ref to position beforehand, so that it must not be done here anymore (a value is expected for every single trajectory point)
     * \param _time_step_size The size of one commonroad timestep (in seconds)
     * \param _id The ID of the simulated obstacle
     * \param _simulated_time Whether simulated time should be used (TODO: Not properly supported /tested yet)
     * \param _custom_stop_signal Custom stop signal for the slow timer that is used when no simulation is performed; Can be used to stop all running obstacle simulations at once & thus to save time when switching to simulation mode
     * \param _get_lanelet_shape Function that returns shape (+ position) of a lanelet (given its ID), used when only a lanelet reference determines an obstacle's position
     */
    ObstacleSimulation(ObstacleSimulationData _trajectory, int _id);

    /**
     * \brief Get the initial state of the obstacles periodically until the simulation is started
     * \param t_now Current time, used for timestamp of msg
     */
    CommonroadObstacle get_init_state(uint64_t t_now);

    /**
     * \brief Send the current obstacle state based on the current time, calculated relative to the start time of the simulation
     * \param start_time Time when the simulation was started
     * \param t_now Current time 
     * \param time_step_size Must be known to find out which current point is active
     */
    CommonroadObstacle get_state(uint64_t start_time, uint64_t t_now, uint64_t time_step_size);

    /**
     * \brief Get the initial trajectory point of the vehicle
     * \param t_now Current time, used for tiemstamp of msg
     */
    //Does not make sense, as with the current implementation the initial trajectory point would not get sent often enough
    //TODO & @Max: We need a "Drive to point" for this, before the simulation starts
    // VehicleCommandTrajectory get_init_trajectory(uint64_t t_now);

    /**
     * \brief Get the current trajectory point of the vehicle, which is interpolated
     * \param start_time Time when the simulation was started
     * \param t_now Current time, used for tiemstamp of msg
     * \param time_step_size Must be known to find out which current point is active
     */
    VehicleCommandTrajectory get_trajectory(uint64_t start_time, uint64_t t_now, uint64_t time_step_size);

    uint8_t get_id();

    //Reset internal counter variable which was implemented to make the lookup a bit faster
    void reset();
};